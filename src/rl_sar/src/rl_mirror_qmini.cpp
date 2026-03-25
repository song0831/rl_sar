/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * rl_mirror_qmini: Joint-mirror sender for real↔sim alignment verification.
 *
 * Runs on Jetson (aarch64) in zero-torque (passive) mode:
 *   - Reads 10-DOF joint angles via QminiMotorController (serial port)
 *   - Reads IMU quaternion + gyroscope via QminiIMU (CP2102 serial)
 *   - Broadcasts joint angles + IMU data via UDP to the development PC
 *   - The PC runs rl_mirror_sim (ROS2 node) which drives Gazebo to follow
 *
 * Usage:
 *   sudo ./cmake_build/bin/rl_mirror_qmini [PC_IP] [PORT]
 *     PC_IP  — target IP address (default: 255.255.255.255 broadcast)
 *     PORT   — UDP port (default: 12345)
 *
 * The tool never enables motor torque — the robot remains fully back-drivable.
 * Manually move each joint and observe whether the Gazebo model follows correctly.
 *
 * UDP packet layout (binary, little-endian, 68 bytes):
 *   uint32_t  magic       = 0x4D495251  ("QMIR")
 *   uint32_t  seq         — frame sequence number
 *   float     q[10]       — joint positions [rad], URDF convention (right-leg sign-flipped)
 *   float     imu_qw, imu_qx, imu_qy, imu_qz  — quaternion (corrected, same as rl_real_qmini)
 *   float     gyr_x, gyr_y, gyr_z              — gyroscope [rad/s] (corrected)
 */

#include "qmini_motor_controller.hpp"

#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

// ── UDP packet ───────────────────────────────────────────────────────────────

static constexpr uint32_t MIRROR_MAGIC = 0x4D495251;  // "QMIR"

#pragma pack(push, 1)
struct MirrorPacket
{
    uint32_t magic = MIRROR_MAGIC;
    uint32_t seq   = 0;
    float    q[10] = {};           // joint positions [rad], URDF convention
    float    imu_qw = 1.f, imu_qx = 0.f, imu_qy = 0.f, imu_qz = 0.f;
    float    gyr_x = 0.f, gyr_y = 0.f, gyr_z = 0.f;
};
#pragma pack(pop)

static_assert(sizeof(MirrorPacket) == 4 + 4 + 10*4 + 4*4 + 3*4, "MirrorPacket size mismatch");

// ── signal handling ──────────────────────────────────────────────────────────

static std::atomic<bool> g_quit{false};
static void sig_handler(int) { g_quit = true; }

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // ── Parse CLI args ──────────────────────────────────────────────────────
    std::string target_ip = "255.255.255.255";  // broadcast by default
    int target_port = 12345;

    if (argc >= 2) target_ip   = argv[1];
    if (argc >= 3) target_port = std::atoi(argv[2]);

    std::printf("[mirror] Target: %s:%d\n", target_ip.c_str(), target_port);

    // ── Load base.yaml for encoder offsets ──────────────────────────────────
    const std::string yaml_path = std::string(POLICY_DIR) + "/Qmini/base.yaml";
    YAML::Node root;
    try { root = YAML::LoadFile(yaml_path); }
    catch (const YAML::BadFile &) {
        std::fprintf(stderr, "[mirror] ERROR: cannot open %s\n", yaml_path.c_str());
        return 1;
    }

    YAML::Node cfg = root["Qmini"];
    if (!cfg) {
        std::fprintf(stderr, "[mirror] ERROR: no 'Qmini' section in %s\n", yaml_path.c_str());
        return 1;
    }

    // Read encoder_offsets
    std::array<float, QminiMotorController::NUM_MOTORS> offsets{};
    if (cfg["encoder_offsets"] && (int)cfg["encoder_offsets"].size() == QminiMotorController::NUM_MOTORS) {
        for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
            offsets[i] = cfg["encoder_offsets"][i].as<float>();
    }

    // Read joint_mapping (should be [0,1,...,9])
    std::array<int, 10> jmap{0,1,2,3,4,5,6,7,8,9};
    if (cfg["joint_mapping"] && (int)cfg["joint_mapping"].size() == 10) {
        for (int i = 0; i < 10; ++i)
            jmap[i] = cfg["joint_mapping"][i].as<int>();
    }

    // Read motor_sign (per-motor direction mapping to URDF convention)
    std::array<float, 10> msign{1,1,1,1,1,1,1,1,1,1};
    if (cfg["motor_sign"] && (int)cfg["motor_sign"].size() == 10) {
        for (int i = 0; i < 10; ++i)
            msign[i] = cfg["motor_sign"][i].as<float>();
    }

    // ── Start motor controller (zero-torque / passive) ──────────────────────
    std::printf("[mirror] Connecting to motors (zero-torque mode)...\n");
    QminiMotorController ctrl(offsets);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    ctrl.printMotorStatus();

    // ── Start IMU ───────────────────────────────────────────────────────────
    QminiIMU imu;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ── Create UDP socket ───────────────────────────────────────────────────
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("[mirror] socket");
        return 1;
    }

    // Enable broadcast
    int broadcast_enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    struct sockaddr_in dest_addr{};
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(target_port);
    inet_aton(target_ip.c_str(), &dest_addr.sin_addr);

    // ── Main loop @ 200 Hz ──────────────────────────────────────────────────
    std::printf("\n[mirror] ========================================\n");
    std::printf("[mirror]  Joint Mirror Mode (zero-torque)\n");
    std::printf("[mirror]  Sending to %s:%d @ 200 Hz\n", target_ip.c_str(), target_port);
    std::printf("[mirror]  Press Ctrl+C to quit.\n");
    std::printf("[mirror] ========================================\n\n");

    MirrorPacket pkt;
    uint32_t seq = 0;
    int print_counter = 0;

    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::milliseconds(5);  // 200 Hz

    while (!g_quit)
    {
        auto t0 = clock::now();

        // Read joint angles (with per-motor sign, same as rl_real_qmini::GetState)
        auto ms = ctrl.getState();
        for (int i = 0; i < 10; ++i)
        {
            int hw_idx = jmap[i];
            float sign = msign[hw_idx];
            pkt.q[i] = sign * ms[hw_idx].q;
        }

        // Read IMU (with Z-180° correction, same as rl_real_qmini::GetState)
        auto imu_data = imu.get();
        if (imu_data.valid)
        {
            float raw_w = imu_data.qw, raw_x = imu_data.qx;
            float raw_y = imu_data.qy, raw_z = imu_data.qz;
            float cw = -raw_z;
            float cx = -raw_y;
            float cy =  raw_x;
            float cz =  raw_w;
            if (cw < 0.f) { cw = -cw; cx = -cx; cy = -cy; cz = -cz; }
            pkt.imu_qw = cw;
            pkt.imu_qx = cx;
            pkt.imu_qy = cy;
            pkt.imu_qz = cz;
            pkt.gyr_x = -imu_data.gyro_y;
            pkt.gyr_y =  imu_data.gyro_x;
            pkt.gyr_z =  imu_data.gyro_z;
        }
        else
        {
            pkt.imu_qw = 1.f; pkt.imu_qx = 0.f; pkt.imu_qy = 0.f; pkt.imu_qz = 0.f;
            pkt.gyr_x = 0.f; pkt.gyr_y = 0.f; pkt.gyr_z = 0.f;
        }

        pkt.seq = seq++;

        // Send UDP
        sendto(sock, &pkt, sizeof(pkt), 0,
               (struct sockaddr *)&dest_addr, sizeof(dest_addr));

        // Print diagnostic at ~2 Hz
        if (++print_counter >= 100)
        {
            print_counter = 0;
            std::printf("\r[mirror] seq=%u  q(rad):", pkt.seq);
            for (int i = 0; i < 10; ++i)
                std::printf(" %6.3f", pkt.q[i]);
            std::printf("  IMU: qw=%5.3f qx=%5.3f qy=%5.3f qz=%5.3f  gyr: %5.2f %5.2f %5.2f",
                        pkt.imu_qw, pkt.imu_qx, pkt.imu_qy, pkt.imu_qz,
                        pkt.gyr_x, pkt.gyr_y, pkt.gyr_z);
            std::fflush(stdout);
        }

        // Rate limiting
        auto elapsed = clock::now() - t0;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }

    close(sock);
    std::printf("\n[mirror] Done.\n");
    return 0;
}
