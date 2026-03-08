/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 *
 * QminiMotorController: Serial-port-based motor driver for Qmini robot.
 * Adapted from QMiniSimAndDeploy/include/user/Motor_thread.hpp
 *
 * Hardware wiring (FTDI USB-Serial, 4 ports):
 *   port0 -> motor IDs {0, 5}   (left/right hip-yaw)
 *   port1 -> motor IDs {1, 6}   (left/right hip-roll)
 *   port2 -> motor IDs {2,3,4}  (left hip-pitch, knee, ankle)
 *   port3 -> motor IDs {7,8,9}  (right hip-pitch, knee, ankle)
 *
 * Motor order in allMotorData[10]:
 *   0=LL_joint1, 1=LL_joint2, 2=LL_joint3, 3=LL_joint4, 4=LL_joint5
 *   5=RL_joint1, 6=RL_joint2, 7=RL_joint3, 8=RL_joint4, 9=RL_joint5
 */

#pragma once

#include <unistd.h>
#include <iostream>
#include <vector>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <cmath>

#include "SerialPort.h"
#include "unitreeMotor.h"

// ---- per-motor state exposed to rl_sar ----
struct QminiMotorState
{
    float q   = 0.f;   // position  [rad], zero-offset removed
    float dq  = 0.f;   // velocity  [rad/s]
    float tau = 0.f;   // torque    [Nm]
};

// ---- per-motor command from rl_sar ----
struct QminiMotorCmd
{
    float q   = 0.f;   // position  [rad], zero-offset removed
    float dq  = 0.f;   // velocity  [rad/s]
    float kp  = 0.f;
    float kd  = 0.f;
    float tau = 0.f;   // feedforward torque [Nm]
};

class QminiMotorController
{
public:
    static constexpr int NUM_MOTORS = 10;

    // Serial port paths: match the FTDI serial-by-id symlinks on Qmini
    // If the FTDI serial number differs from the reference machine,
    // fall back to /dev/ttyUSBx ordered by enumeration.
    static constexpr const char *FTDI_ID = "usb-FTDI_USB__-__Serial_Converter_FTANCW0H";

    struct SerialGroup
    {
        std::string port;
        std::vector<int> motorIDs;
    };

    // Mechanical zero offsets [rad] in SDK/hardware order (motor 0..9)
    // Adapted from QMiniSimAndDeploy Startq array
    const std::array<float, NUM_MOTORS> startq =
        {0.10f, 0.05f, 2.07f, 0.01f, 1.60f,
         1.14f, 0.32f,-0.88f, 1.29f,-0.87f};

    // Gear ratios
    const float speed_ratio = 6.33f;
    const float gear_ratio  = 3.0f;   // only for special motors (IDs 1 & 6)

    QminiMotorController()
    {
        // Resolve serial port paths (prefer by-id, fall back to ttyUSB)
        resolveSerialPorts();
        initializeSerialPorts();

        // Start one thread per serial port group
        running_ = true;
        for (int i = 0; i < 4; ++i)
            worker_threads_[i] = std::thread(&QminiMotorController::runThread, this, i);

        std::cout << "[QminiMotorController] Serial motor threads started." << std::endl;
    }

    ~QminiMotorController()
    {
        running_ = false;
        for (auto &t : worker_threads_)
            if (t.joinable()) t.join();
    }

    // Called once (e.g. when user presses 'A') to allow torque commands.
    // Before this, runThread still does sendRecv (to keep the motor comms
    // alive and read encoder feedback) but forces kp=kd=tau=0.
    void enableMotors()
    {
        motors_enabled_ = true;
        std::cout << "[QminiMotorController] Motors ENABLED (torque allowed)." << std::endl;
    }

    bool isEnabled() const { return motors_enabled_.load(); }

    // Called by rl_sar at ~500 Hz to push new commands
    void setCommand(const std::array<QminiMotorCmd, NUM_MOTORS> &cmd)
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        pending_cmd_ = cmd;
    }

    // Called by rl_sar to read latest feedback
    std::array<QminiMotorState, NUM_MOTORS> getState() const
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        return motor_state_;
    }

private:
    std::vector<SerialGroup>                serial_groups_;
    std::vector<std::unique_ptr<SerialPort>> serial_ports_;

    std::array<QminiMotorState, NUM_MOTORS> motor_state_{};
    std::array<QminiMotorCmd,   NUM_MOTORS> pending_cmd_{};
    mutable std::mutex state_mutex_;
    mutable std::mutex cmd_mutex_;

    std::atomic<bool> running_{false};
    std::atomic<bool> motors_enabled_{false};  // safety gate: no torque until armed
    std::array<std::thread, 4> worker_threads_;

    // Motor IDs 1 and 6 have an extra gear stage
    bool isSpecialMotor(int id) const { return id == 1 || id == 6; }

    float motorRatio(int id) const
    {
        return isSpecialMotor(id) ? speed_ratio * gear_ratio : speed_ratio;
    }

    // Channel ID within the serial group (mirrors CalculateChannelID in Motor_thread.hpp)
    // Group if00: motorIDs {0, 5}    → channelID 0, 1
    // Group if01: motorIDs {1, 6}    → channelID 0, 1  (motor 1 is special: id-1; motor 6: id-5)
    // Group if02: motorIDs {2, 3, 4} → channelID 0, 1, 2 (id-2)
    // Group if03: motorIDs {7, 8, 9} → channelID 0, 1, 2 (id-7)
    int channelID(int motorID) const
    {
        if (motorID == 0)                         return 0;   // if00 first  motor
        if (motorID == 5)                         return 1;   // if00 second motor  ← was missing!
        if (motorID == 1)                         return 0;   // if01 first  motor (id-1)
        if (motorID == 6)                         return 1;   // if01 second motor (id-5)  ← was returning 0, wrong!
        if (motorID >= 2 && motorID <= 4)         return motorID - 2;  // if02: 0,1,2
        if (motorID >= 7 && motorID <= 9)         return motorID - 7;  // if03: 0,1,2
        return motorID;
    }

    void resolveSerialPorts()
    {
        // Try by-id first (stable across reboots)
        std::string base = std::string("/dev/serial/by-id/") + FTDI_ID;
        auto tryPort = [&](const std::string &suffix) -> std::string {
            std::string p = base + suffix;
            if (access(p.c_str(), F_OK) == 0) return p;
            return "";
        };

        std::array<std::string, 4> ports = {
            tryPort("-if00-port0"),
            tryPort("-if01-port0"),
            tryPort("-if02-port0"),
            tryPort("-if03-port0"),
        };

        // Fall back to /dev/ttyUSB1~4 if by-id not found
        const char *fallback[4] = {"/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4"};
        for (int i = 0; i < 4; ++i)
            if (ports[i].empty()) ports[i] = fallback[i];

        serial_groups_ = {
            {ports[0], {0, 5}},
            {ports[1], {1, 6}},
            {ports[2], {2, 3, 4}},
            {ports[3], {7, 8, 9}},
        };

        for (int i = 0; i < 4; ++i)
            std::cout << "[QminiMotorController] Group " << i
                      << " -> " << serial_groups_[i].port << std::endl;
    }

    void initializeSerialPorts()
    {
        for (auto &g : serial_groups_)
            serial_ports_.push_back(std::make_unique<SerialPort>(g.port.c_str()));
    }

    void runThread(int groupIdx)
    {
        SerialPort &serial = *serial_ports_[groupIdx];
        const std::vector<int> &ids = serial_groups_[groupIdx].motorIDs;

        // Rate: each group has 2-3 motors; target ~200 Hz per group
        // so overall control bandwidth is ~200 Hz per motor.
        // A 2 ms sleep per iteration limits max rate to ~500 Hz.
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::microseconds(2000); // 2 ms = 500 Hz

        while (running_)
        {
            auto t0 = clock::now();

            // Snapshot current command
            std::array<QminiMotorCmd, NUM_MOTORS> cmd;
            {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                cmd = pending_cmd_;
            }

            const bool armed = motors_enabled_.load();

            for (int motorID : ids)
            {
                MotorCmd mc;
                MotorData md;

                const QminiMotorCmd &c = cmd[motorID];
                const float ratio = motorRatio(motorID);

                mc.motorType = MotorType::GO_M8010_6;
                mc.mode      = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                mc.id        = channelID(motorID);

                if (armed)
                {
                    // Convert joint-space kp/kd → motor-axis kp/kd
                    // The motor protocol uses motor-axis coordinates:
                    //   tau = kp*(q_cmd - q) + kd*(dq_cmd - dq)   [all in motor-axis rad]
                    // But rl_sar passes joint-space kp/kd (same as sim).
                    // Relation: kp_motor = kp_joint / ratio^2
                    //           kd_motor = kd_joint / ratio^2
                    mc.kp  = c.kp  / (ratio * ratio);
                    mc.kd  = c.kd  / (ratio * ratio);
                    mc.tau = c.tau;
                    mc.q   = (c.q + startq[motorID]) * ratio;
                    mc.dq  = c.dq * ratio;
                }
                else
                {
                    // Safety mode: zero torque, track current position
                    // (send zero kp/kd/tau so motors are back-drivable)
                    mc.kp  = 0.f;
                    mc.kd  = 0.f;
                    mc.tau = 0.f;
                    mc.q   = 0.f;
                    mc.dq  = 0.f;
                }

                md.motorType = MotorType::GO_M8010_6;
                serial.sendRecv(&mc, &md);

                // Parse feedback (always, regardless of armed state)
                QminiMotorState s;
                s.q   = md.q   / ratio - startq[motorID];
                s.dq  = md.dq  / ratio;
                s.tau = md.tau;

                {
                    std::lock_guard<std::mutex> lk(state_mutex_);
                    motor_state_[motorID] = s;
                }
            }

            // Rate limiting: sleep for remainder of period
            auto elapsed = clock::now() - t0;
            if (elapsed < period)
                std::this_thread::sleep_for(period - elapsed);
        }
    }
};

// ---------------------------------------------------------------------------
// QminiIMU — C++ driver for the CP2102-based IMU on /dev/ttyUSB4
//
// Protocol (AHRS Tech / HIPNUC):
//   Frame header : 0xFC
//   TYPE_IMU  = 0x40  (accelerometer, 56 bytes payload)
//   TYPE_AHRS = 0x41  (roll/pitch/yaw + quaternion + gyro, 48 bytes payload)
//
// We parse TYPE_AHRS frames only.
// Payload layout (TYPE_AHRS, 48 bytes = 10 floats + 2 int32):
//   [0] RollSpeed  [rad/s]
//   [1] PitchSpeed [rad/s]
//   [2] HeadingSpeed [rad/s]
//   [3] Roll  [rad]
//   [4] Pitch [rad]
//   [5] Heading [rad]
//   [6] qw
//   [7] qx
//   [8] qy
//   [9] qz
//   [10][11] int32 (unused)
// ---------------------------------------------------------------------------
class QminiIMU
{
public:
    // CP2102 IMU — prefer by-id path for stability
    static constexpr const char *IMU_DEV_BYID =
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
    static constexpr const char *IMU_DEV_FALLBACK = "/dev/ttyUSB4";
    static constexpr int IMU_BAUDRATE    = 921600;

    struct Data
    {
        float qw = 1.f, qx = 0.f, qy = 0.f, qz = 0.f;
        float gyro_x = 0.f, gyro_y = 0.f, gyro_z = 0.f;
        bool  valid  = false;
    };

    QminiIMU()
    {
        // Try by-id first (stable across reboots), fall back to ttyUSB4
        fd_ = openSerial(IMU_DEV_BYID, IMU_BAUDRATE);
        const char *used_dev = IMU_DEV_BYID;
        if (fd_ < 0)
        {
            std::cerr << "[QminiIMU] by-id open failed (errno=" << errno
                      << "), trying fallback " << IMU_DEV_FALLBACK << std::endl;
            fd_ = openSerial(IMU_DEV_FALLBACK, IMU_BAUDRATE);
            used_dev = IMU_DEV_FALLBACK;
        }
        if (fd_ < 0)
        {
            std::cerr << "[QminiIMU] Failed to open IMU port (errno=" << errno << ")" << std::endl;
            return;
        }
        running_ = true;
        thread_  = std::thread(&QminiIMU::readLoop, this);
        std::cout << "[QminiIMU] Started reading from " << used_dev << std::endl;
    }

    ~QminiIMU()
    {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) close(fd_);
    }

    Data get() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return data_;
    }

private:
    int         fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    Data        data_;

    static int openSerial(const char *dev, int baud)
    {
        int fd = ::open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return -1;
        // switch to blocking
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

        // Read existing settings first, then modify — avoids zeroing hidden fields
        // that tcsetattr may refuse to apply on Jetson (aarch64 termios2 quirk)
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
            ::memset(&tty, 0, sizeof tty);

        cfsetispeed(&tty, B921600);
        cfsetospeed(&tty, B921600);
        // c_cflag: keep baud bits, set 8N1 + CLOCAL + CREAD
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |=  CS8 | CLOCAL | CREAD;
        tty.c_iflag  =  IGNPAR;
        tty.c_oflag  =  0;
        tty.c_lflag  =  0;
        tty.c_cc[VTIME] = 10; // 1s timeout (units: 100ms)
        tty.c_cc[VMIN]  = 1;  // block until at least 1 byte received
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &tty);
        // Verify VMIN actually stuck (Jetson quirk: re-apply if needed)
        struct termios verify;
        tcgetattr(fd, &verify);
        if (verify.c_cc[VMIN] != 1)
        {
            verify.c_cc[VMIN]  = 1;
            verify.c_cc[VTIME] = 10;
            tcsetattr(fd, TCSANOW, &verify);
        }
        return fd;
    }

    // Read exactly n bytes (blocking with timeout)
    bool readBytes(uint8_t *buf, int n)
    {
        int got = 0;
        while (got < n && running_)
        {
            int r = ::read(fd_, buf + got, n - got);
            if (r > 0)
            {
                got += r;
            }
            else if (r == 0)
            {
                // Timeout (VTIME) — no data yet, keep waiting
                continue;
            }
            else // r < 0
            {
                if (errno == EINTR || errno == EAGAIN) continue;
                return false; // real error
            }
        }
        return got == n;
    }

    void readLoop()
    {
        static constexpr uint8_t FRAME_HEAD = 0xFC;
        static constexpr uint8_t TYPE_IMU   = 0x40;
        static constexpr uint8_t TYPE_AHRS  = 0x41;
        static constexpr uint8_t AHRS_LEN   = 0x30; // 48 bytes
        static constexpr uint8_t IMU_LEN    = 0x38; // 56 bytes

        int dbg_bytes = 0, dbg_syncs = 0, dbg_frames = 0;

        while (running_)
        {
            // Sync to frame header
            uint8_t b;
            if (!readBytes(&b, 1)) continue;
            dbg_bytes++;
            if (b != FRAME_HEAD) continue;
            dbg_syncs++;

            uint8_t head_type, check_len;
            if (!readBytes(&head_type, 1)) continue;
            if (!readBytes(&check_len, 1)) continue;

            // Validate known types
            if (head_type == TYPE_AHRS && check_len != AHRS_LEN) continue;
            if (head_type == TYPE_IMU  && check_len != IMU_LEN)  continue;
            if (head_type != TYPE_AHRS && head_type != TYPE_IMU)
            {
                // Skip unknown frame: read check_len bytes + 4 header bytes
                uint8_t skip[256];
                readBytes(skip, (int)check_len + 4);
                continue;
            }

            uint8_t sn, crc8, crc16h, crc16l;
            if (!readBytes(&sn,    1)) continue;
            if (!readBytes(&crc8,  1)) continue;
            if (!readBytes(&crc16h,1)) continue;
            if (!readBytes(&crc16l,1)) continue;

            if (head_type == TYPE_AHRS)
            {
                uint8_t payload[48];
                if (!readBytes(payload, 48)) continue;
                // 10 floats + 2 int32  (little-endian)
                float vals[10];
                for (int i = 0; i < 10; ++i)
                    ::memcpy(&vals[i], payload + i * 4, 4);

                std::lock_guard<std::mutex> lk(mutex_);
                data_.gyro_x = vals[0];
                data_.gyro_y = vals[1];
                data_.gyro_z = vals[2];
                data_.qw     = vals[6];
                data_.qx     = vals[7];
                data_.qy     = vals[8];
                data_.qz     = vals[9];
                data_.valid  = true;
                dbg_frames++;
                if (dbg_frames <= 3)
                    std::printf("[QminiIMU] AHRS frame #%d: qw=%.3f qx=%.3f qy=%.3f qz=%.3f (bytes=%d syncs=%d)\n",
                        dbg_frames, vals[6], vals[7], vals[8], vals[9], dbg_bytes, dbg_syncs);
            }
            else if (head_type == TYPE_IMU)
            {
                uint8_t payload[56];
                readBytes(payload, 56); // discard, we only need AHRS
            }
        }
    }
};
