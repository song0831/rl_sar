/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 *
 * rl_calib_qmini: Encoder zero-offset calibration & joint-limit scan tool for Qmini robot.
 *
 * Usage:
 *   sudo ./cmake_build/bin/rl_calib_qmini
 *
 * Mode 1 — Encoder Zero-Offset Calibration:
 *   1. Place the robot in a stable standard standing pose (matching default_dof_pos).
 *   2. Run this tool — it connects to motors in zero-torque (passive) mode.
 *   3. Press Enter to confirm the robot is at the standard pose.
 *   4. New encoder_offsets are computed and written back to base.yaml automatically.
 *   Formula:  new_offset[i] = old_offset[i] + (current_q[i] - default_dof_pos[i])
 *
 * Mode 2 — Joint Limit Scan:
 *   1. Select a joint by index (0-9).
 *   2. Manually move the joint to its MINIMUM limit, press Enter to record.
 *   3. Manually move the joint to its MAXIMUM limit, press Enter to record.
 *   4. Repeat for as many joints as needed.
 *   5. Press 's' to save all recorded limits to base.yaml and sync to Qmini-orin.
 *
 * After calibration, restart rl_real_qmini to load the updated values.
 */

#include "qmini_motor_controller.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// ── helpers ──────────────────────────────────────────────────────────────────

static const char *MOTOR_NAMES[10] = {
    "M0 L-hip-yaw",  "M1 L-hip-roll", "M2 L-hip-pitch",
    "M3 L-knee",     "M4 L-ankle",
    "M5 R-hip-yaw",  "M6 R-hip-roll", "M7 R-hip-pitch",
    "M8 R-knee",     "M9 R-ankle"
};

static bool g_quit = false;
static void sig_handler(int) { g_quit = true; }

// ── average N samples ────────────────────────────────────────────────────────
// Returns {ok, result}.
//   ok = false -> at least one motor was offline OR
//                 any joint stddev > max_stddev_rad (unstable comms / moving joint).
// The caller should abort and NOT write yaml on failure.
static std::pair<bool, std::array<float, QminiMotorController::NUM_MOTORS>>
    sampleAvg(QminiMotorController &ctrl, int n = 40, float max_stddev_rad = 0.03f)
{
    constexpr int N = QminiMotorController::NUM_MOTORS;

    // Wait up to 3 s for every motor to come online
    for (int wait = 0; wait < 30; ++wait)
    {
        auto status = ctrl.getMotorStatus();
        bool all_ok = true;
        for (int i = 0; i < N; ++i) if (!status[i]) { all_ok = false; break; }
        if (all_ok) break;
        if (wait == 29)
        {
            std::printf("[calib] ERROR: not all motors online before sampling.\n");
            std::printf("[calib]   Motor status:");
            for (int i = 0; i < N; ++i)
                std::printf(" M%d:%s", i, status[i] ? "OK" : "FAIL");
            std::printf("\n");
            return {false, {}};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Collect n samples
    std::array<double, N> sum{};
    std::array<double, N> sum2{};
    for (int s = 0; s < n; ++s)
    {
        auto ms = ctrl.getState();
        for (int i = 0; i < N; ++i)
        {
            double v = ms[i].q;
            sum[i]  += v;
            sum2[i] += v * v;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Compute mean + stddev; report any high-variance joint
    std::array<float, N> result{};
    bool stable = true;
    for (int i = 0; i < N; ++i)
    {
        double mean   = sum[i]  / n;
        double var    = sum2[i] / n - mean * mean;
        double stddev = std::sqrt(std::max(var, 0.0));
        result[i] = static_cast<float>(mean);
        if (stddev > max_stddev_rad)
        {
            std::printf("[calib] WARNING: M%d stddev=%.4f rad > %.4f "
                        "(unstable comms or joint still moving?).\n",
                        i, stddev, max_stddev_rad);
            stable = false;
        }
    }
    if (!stable)
        std::printf("[calib] WARNING: high variance -- check serial cable, "
                    "hold joints still, and retry.\n");
    return {stable, result};
}

// ── write yaml to disk ────────────────────────────────────────────────────────
static bool writeYaml(const YAML::Node &root, const std::string &path)
{
    YAML::Emitter out;
    out << root;
    FILE *fp = std::fopen(path.c_str(), "w");
    if (!fp)
    {
        std::perror("[calib] ERROR: cannot open yaml for writing");
        return false;
    }
    std::fputs(out.c_str(), fp);
    std::fclose(fp);
    return true;
}

// ── live display thread (shows all joint angles) ──────────────────────────────
static void livePrintLoop(QminiMotorController &ctrl,
                          const std::optional<int> &highlight_joint,
                          const std::array<float, QminiMotorController::NUM_MOTORS> &motor_sign)
{
    while (!g_quit)
    {
        auto ms = ctrl.getState();
        std::printf("\r  q[rad]: ");
        for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
        {
            float q = motor_sign[i] * ms[i].q;
            if (highlight_joint.has_value() && i == *highlight_joint)
                std::printf("\033[1;33m%7.3f\033[0m", q);  // yellow highlight
            else
                std::printf("%7.3f", q);
        }
        std::printf("  ");
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── Mode 1: encoder zero-offset calibration ──────────────────────────────────
static int runOffsetCalib(QminiMotorController &ctrl,
                          YAML::Node &root,
                          const std::string &yaml_path,
                          const std::array<float, QminiMotorController::NUM_MOTORS> &old_offsets,
                          const std::array<float, QminiMotorController::NUM_MOTORS> &default_dof_pos,
                          const std::array<float, QminiMotorController::NUM_MOTORS> &motor_sign)
{
    std::cout << "\n[calib] Current encoder_offsets loaded from yaml:\n  [ ";
    for (float v : old_offsets) std::cout << v << " ";
    std::cout << "]\n";

    std::cout << "\n[calib] Expected default_dof_pos:\n  [ ";
    for (float v : default_dof_pos) std::cout << v << " ";
    std::cout << "]\n";

    std::cout << "\n[calib] ----------------------------------------------------\n";
    std::cout << "[calib] Manually move the robot to the standard standing pose.\n";
    std::cout << "[calib] Press  ENTER  to capture joint positions.\n";
    std::cout << "[calib] Press  Ctrl+C to abort without saving.\n";
    std::cout << "[calib] ----------------------------------------------------\n\n";
    std::cout << "Live joint angles (q[rad]):\n";

    // Live display while waiting for Enter
    std::optional<int> no_highlight;
    std::thread live_thread([&](){ livePrintLoop(ctrl, no_highlight, motor_sign); });

    std::cin.get();
    g_quit = true;
    live_thread.join();
    std::cout << "\n";

    if (!std::cin)
    {
        std::cout << "[calib] Aborted by user. No changes written.\n";
        return 0;
    }

    // Capture current joint angles (40 samples with communication-quality checks)
    auto [sample_ok, current_q_raw] = sampleAvg(ctrl);
    if (!sample_ok)
    {
        std::cout << "[calib] Sampling failed (comms unstable or motor offline).\n"
                  << "[calib] No changes written. Check serial cable and retry.\n";
        return 1;
    }

    // Apply per-motor sign to convert raw encoder reading to URDF frame.
    std::array<float, QminiMotorController::NUM_MOTORS> current_q{};
    for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
        current_q[i] = motor_sign[i] * current_q_raw[i];

    // Compute new offsets: new_offset[i] = old_offset[i] + (current_q[i] - default_dof_pos[i])
    // Note: both current_q and default_dof_pos are in URDF frame.
    // encoder_offset lives in raw hardware frame, so delta must be converted back:
    //   raw_delta = motor_sign[i] * (current_q[i] - default_dof_pos[i])
    // (since motor_sign is ±1, multiplying again reverses the conversion)
    std::array<float, QminiMotorController::NUM_MOTORS> new_offsets{};
    for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
    {
        float delta_urdf = current_q[i] - default_dof_pos[i];
        float delta_raw  = motor_sign[i] * delta_urdf;
        new_offsets[i] = old_offsets[i] + delta_raw;
    }

    // Print summary
    std::cout << "\n[calib] Calibration result:\n";
    std::cout << "  " << std::string(72, '-') << "\n";
    std::printf("  %-20s  %8s  %8s  %8s  %8s\n",
                "Motor", "old_off", "curr_q", "def_q", "new_off");
    std::cout << "  " << std::string(72, '-') << "\n";
    for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
    {
        float delta = current_q[i] - default_dof_pos[i];
        std::printf("  %-20s  %8.4f  %8.4f  %8.4f  %8.4f  (delta %+.4f)\n",
                    MOTOR_NAMES[i],
                    old_offsets[i], current_q[i], default_dof_pos[i],
                    new_offsets[i], delta);
    }
    std::cout << "  " << std::string(72, '-') << "\n";

    std::cout << "\n[calib] Write new encoder_offsets to " << yaml_path << "? [y/N] ";
    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "y" && answer != "Y")
    {
        std::cout << "[calib] Aborted. No changes written.\n";
        return 0;
    }

    // Write back to yaml
    YAML::Node new_seq;
    for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
        new_seq.push_back(new_offsets[i]);
    root["Qmini"]["encoder_offsets"] = new_seq;

    if (!writeYaml(root, yaml_path))
        return 1;

    std::cout << "[calib] ✓ encoder_offsets saved to " << yaml_path << "\n";
    std::cout << "[calib] Restart rl_real_qmini to apply the new calibration.\n\n";
    return 0;
}

// ── Mode 2: joint limit scan ─────────────────────────────────────────────────
static int runLimitScan(QminiMotorController &ctrl,
                        YAML::Node &root,
                        const std::string &yaml_path,
                        const std::array<float, QminiMotorController::NUM_MOTORS> &motor_sign)
{
    constexpr int N = QminiMotorController::NUM_MOTORS;

    // Load existing limits from yaml (if present)
    const float UNSET = std::numeric_limits<float>::quiet_NaN();
    std::array<float, N> lim_min{}, lim_max{};
    lim_min.fill(UNSET);
    lim_max.fill(UNSET);

    YAML::Node cfg = root["Qmini"];
    if (cfg["dof_pos_limits_min"] && (int)cfg["dof_pos_limits_min"].size() == N)
        for (int i = 0; i < N; ++i)
            lim_min[i] = cfg["dof_pos_limits_min"][i].as<float>();
    if (cfg["dof_pos_limits_max"] && (int)cfg["dof_pos_limits_max"].size() == N)
        for (int i = 0; i < N; ++i)
            lim_max[i] = cfg["dof_pos_limits_max"][i].as<float>();

    // Helper: print full limit table
    auto printLimits = [&]()
    {
        std::cout << "\n  Current recorded limits:\n";
        std::cout << "  " << std::string(62, '-') << "\n";
        std::printf("  %-4s  %-20s  %10s  %10s  %8s\n",
                    "Idx", "Motor", "min[rad]", "max[rad]", "range");
        std::cout << "  " << std::string(62, '-') << "\n";
        for (int i = 0; i < N; ++i)
        {
            bool set = !std::isnan(lim_min[i]) && !std::isnan(lim_max[i]);
            std::printf("  [%d]   %-20s  %10s  %10s  %8s\n",
                        i, MOTOR_NAMES[i],
                        std::isnan(lim_min[i]) ? "(unset)" : (std::to_string(lim_min[i]).substr(0,8)).c_str(),
                        std::isnan(lim_max[i]) ? "(unset)" : (std::to_string(lim_max[i]).substr(0,8)).c_str(),
                        set ? (std::to_string(lim_max[i]-lim_min[i]).substr(0,7)).c_str() : "-");
        }
        std::cout << "  " << std::string(62, '-') << "\n";
    };

    std::cout << "\n========================================================\n";
    std::cout << "  Joint Limit Scan Mode  (auto min/max tracking)\n";
    std::cout << "  All joints are in PASSIVE (zero-torque) mode.\n";
    std::cout << "  Select a joint, then freely rotate it across its full\n";
    std::cout << "  range — min & max are tracked automatically.\n";
    std::cout << "  Press ENTER to confirm, 'c'+ENTER to discard.\n";
    std::cout << "  NOTE: motor_sign from base.yaml is applied\n";
    std::cout << "        to convert to URDF/RL convention.\n";
    std::cout << "========================================================\n";

    printLimits();

    while (true)
    {
        std::cout << "\n[limit] Commands:\n";
        std::cout << "  <0-9>  — select joint (auto-tracks min/max while you move it)\n";
        std::cout << "  s      — save all limits to base.yaml\n";
        std::cout << "  p      — print current limit table\n";
        std::cout << "  q      — quit without saving\n";
        std::cout << "[limit] Enter command: ";

        std::string line;
        if (!std::getline(std::cin, line) || g_quit)
        {
            std::cout << "\n[limit] Aborted.\n";
            return 0;
        }
        if (line.empty()) continue;

        if (line == "q" || line == "Q")
        {
            std::cout << "[limit] Quit without saving.\n";
            return 0;
        }

        if (line == "p" || line == "P")
        {
            printLimits();
            continue;
        }

        if (line == "s" || line == "S")
        {
            bool any_unset = false;
            for (int i = 0; i < N; ++i)
                if (std::isnan(lim_min[i]) || std::isnan(lim_max[i])) { any_unset = true; break; }
            if (any_unset)
                std::cout << "[limit] WARNING: some joints have unset limits (will be saved as 0.0).\n";

            printLimits();
            std::cout << "\n[limit] Save limits to " << yaml_path << "? [y/N] ";
            std::string ans;
            std::getline(std::cin, ans);
            if (ans != "y" && ans != "Y") { std::cout << "[limit] Not saved.\n"; continue; }

            YAML::Node seq_min, seq_max;
            for (int i = 0; i < N; ++i)
            {
                seq_min.push_back(std::isnan(lim_min[i]) ? 0.f : lim_min[i]);
                seq_max.push_back(std::isnan(lim_max[i]) ? 0.f : lim_max[i]);
            }
            root["Qmini"]["dof_pos_limits_min"] = seq_min;
            root["Qmini"]["dof_pos_limits_max"] = seq_max;

            if (!writeYaml(root, yaml_path))
                return 1;
            std::cout << "[limit] ✓ Limits saved to " << yaml_path << "\n";
            return 0;
        }

        // ── Joint index selection ────────────────────────────────────────
        if (line.size() != 1 || !std::isdigit((unsigned char)line[0]))
        {
            std::cout << "[limit] Unknown command '" << line << "'\n";
            continue;
        }

        int jidx = line[0] - '0';
        if (jidx >= N)
        {
            std::cout << "[limit] Invalid joint index " << jidx
                      << ". Must be 0–" << (N - 1) << ".\n";
            continue;
        }

        std::printf("\n[limit] Joint [%d] %s — rotate freely across full range.\n",
                    jidx, MOTOR_NAMES[jidx]);
        std::printf("[limit] Previous:  min=%-9s  max=%s\n",
                    std::isnan(lim_min[jidx]) ? "(unset)" : std::to_string(lim_min[jidx]).substr(0,8).c_str(),
                    std::isnan(lim_max[jidx]) ? "(unset)" : std::to_string(lim_max[jidx]).substr(0,8).c_str());
        std::cout << "[limit] Tracking started. Press ENTER to confirm, 'c'+ENTER to discard.\n\n";

        // ── Auto min/max tracking thread ─────────────────────────────────
        // Shared tracking state (updated by background thread, read by display)
        float track_min =  std::numeric_limits<float>::max();
        float track_max = -std::numeric_limits<float>::max();
        std::atomic<bool> tracking{true};

        // Background thread: sample motor at ~200 Hz, update min/max
        // Apply motor_sign so tracked values are in URDF coordinate frame.
        std::thread t_track([&]()
        {
            while (tracking.load())
            {
                auto ms = ctrl.getState();
                float q = motor_sign[jidx] * ms[jidx].q;
                if (q < track_min) track_min = q;
                if (q > track_max) track_max = q;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });

        // Display thread: refresh line every 100 ms
        g_quit = false;
        std::thread t_disp([&]()
        {
            while (!g_quit)
            {
                auto ms = ctrl.getState();
                float q = motor_sign[jidx] * ms[jidx].q;
                // highlight current value in yellow, show live min/max
                std::printf("\r  \033[1;33mq=%7.3f\033[0m  min=\033[1;36m%7.3f\033[0m"
                            "  max=\033[1;35m%7.3f\033[0m  range=%6.3f   ",
                            q, track_min, track_max,
                            (track_max > track_min) ? (track_max - track_min) : 0.f);
                std::fflush(stdout);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

        // Wait for user input
        std::string inp;
        std::getline(std::cin, inp);

        // Stop both threads
        g_quit = true;
        tracking.store(false);
        t_disp.join();
        t_track.join();
        g_quit = false;
        std::cout << "\n";

        if (inp == "c" || inp == "C")
        {
            std::cout << "[limit] Discarded. Joint [" << jidx << "] limits unchanged.\n";
            continue;
        }

        // Sanity check: must have actually moved
        if (track_min == std::numeric_limits<float>::max())
        {
            std::cout << "[limit] No data captured. Skipping.\n";
            continue;
        }

        if (track_max < track_min + 0.01f)
            std::printf("[limit] WARNING: range only %.4f rad — did you move the joint?\n",
                        track_max - track_min);

        // track_min/max are already in URDF coordinate frame (sign flip applied
        // during tracking above), so store directly.
        lim_min[jidx] = track_min;
        lim_max[jidx] = track_max;

        std::printf("[limit] ✓ Joint [%d] %-20s  min=%8.4f  max=%8.4f  range=%.4f rad\n",
                    jidx, MOTOR_NAMES[jidx], lim_min[jidx], lim_max[jidx],
                    lim_max[jidx] - lim_min[jidx]);
    }
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int /*argc*/, char ** /*argv*/)
{
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // ── 1. Load base.yaml ───────────────────────────────────────────────────
    const std::string yaml_path = std::string(POLICY_DIR) + "/Qmini/base.yaml";

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(yaml_path);
    }
    catch (const YAML::BadFile &)
    {
        std::cerr << "[calib] ERROR: cannot open " << yaml_path << "\n";
        return 1;
    }

    YAML::Node cfg = root["Qmini"];
    if (!cfg)
    {
        std::cerr << "[calib] ERROR: no 'Qmini' section found in " << yaml_path << "\n";
        return 1;
    }

    // Read current encoder_offsets
    if (!cfg["encoder_offsets"])
    {
        std::cerr << "[calib] ERROR: 'encoder_offsets' key missing in base.yaml\n";
        return 1;
    }
    std::array<float, QminiMotorController::NUM_MOTORS> old_offsets{};
    {
        auto node = cfg["encoder_offsets"];
        if ((int)node.size() != QminiMotorController::NUM_MOTORS)
        {
            std::cerr << "[calib] ERROR: encoder_offsets must have "
                      << QminiMotorController::NUM_MOTORS << " elements\n";
            return 1;
        }
        for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
            old_offsets[i] = node[i].as<float>();
    }

    // Read default_dof_pos
    if (!cfg["default_dof_pos"])
    {
        std::cerr << "[calib] ERROR: 'default_dof_pos' key missing in base.yaml\n";
        return 1;
    }
    std::array<float, QminiMotorController::NUM_MOTORS> default_dof_pos{};
    {
        auto node = cfg["default_dof_pos"];
        if ((int)node.size() != QminiMotorController::NUM_MOTORS)
        {
            std::cerr << "[calib] ERROR: default_dof_pos must have "
                      << QminiMotorController::NUM_MOTORS << " elements\n";
            return 1;
        }
        for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
            default_dof_pos[i] = node[i].as<float>();
    }

    // Read motor_sign (per-motor direction mapping to URDF convention)
    std::array<float, QminiMotorController::NUM_MOTORS> motor_sign{};
    motor_sign.fill(1.f);
    if (cfg["motor_sign"] && (int)cfg["motor_sign"].size() == QminiMotorController::NUM_MOTORS)
    {
        for (int i = 0; i < QminiMotorController::NUM_MOTORS; ++i)
            motor_sign[i] = cfg["motor_sign"][i].as<float>();
    }
    else
    {
        std::cout << "[calib] WARNING: motor_sign not found in yaml, defaulting to all +1.\n";
    }

    // ── 2. Start motor controller (passive / zero-torque) ───────────────────
    std::cout << "\n[calib] Connecting to motors (zero-torque mode)...\n";
    QminiMotorController ctrl(old_offsets);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    ctrl.printMotorStatus();

    // ── 3. Mode selection ───────────────────────────────────────────────────
    std::cout << "\n========================================================\n";
    std::cout << "  rl_calib_qmini — Calibration Tool\n";
    std::cout << "========================================================\n";
    std::cout << "  [1]  Encoder zero-offset calibration  (standing pose)\n";
    std::cout << "  [2]  Joint limit scan  (min/max mechanical limits)\n";
    std::cout << "  [q]  Quit\n";
    std::cout << "========================================================\n";
    std::cout << "Select mode: ";

    std::string mode_str;
    if (!std::getline(std::cin, mode_str) || mode_str == "q" || mode_str == "Q")
    {
        std::cout << "[calib] Bye.\n";
        return 0;
    }

    if (mode_str == "1")
    {
        return runOffsetCalib(ctrl, root, yaml_path, old_offsets, default_dof_pos, motor_sign);
    }
    else if (mode_str == "2")
    {
        return runLimitScan(ctrl, root, yaml_path, motor_sign);
    }
    else
    {
        std::cout << "[calib] Unknown mode '" << mode_str << "'. Exiting.\n";
        return 1;
    }
}