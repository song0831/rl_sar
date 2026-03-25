/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * rl_mirror_sim: Gazebo-side joint mirror receiver for real↔sim alignment verification.
 *
 * Receives joint angles + IMU data from the real Qmini robot (sent by rl_mirror_qmini
 * running on Jetson via UDP), and drives the Gazebo simulation model to follow.
 *
 * This allows visual comparison of:
 *   1. Joint angle mapping — does each URDF joint match the real motor?
 *   2. Joint sign convention — are left/right legs correctly flipped?
 *   3. IMU alignment — does the real IMU orientation match the Gazebo IMU?
 *
 * Usage:
 *   # Terminal 1: launch Gazebo
 *   source install/setup.bash
 *   ros2 launch rl_sar gazebo.launch.py rname:=Qmini
 *
 *   # Terminal 2: run mirror receiver
 *   source install/setup.bash
 *   ros2 run rl_sar rl_mirror_sim [PORT]
 *     PORT — UDP listen port (default: 12345, must match rl_mirror_qmini)
 *
 * No policy inference is performed — pure passthrough of real joint angles to Gazebo.
 */

#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/wait.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ── UDP packet (must match rl_mirror_qmini.cpp) ─────────────────────────────

static constexpr uint32_t MIRROR_MAGIC = 0x4D495251;  // "QMIR"

#pragma pack(push, 1)
struct MirrorPacket
{
    uint32_t magic = MIRROR_MAGIC;
    uint32_t seq   = 0;
    float    q[10] = {};
    float    imu_qw = 1.f, imu_qx = 0.f, imu_qy = 0.f, imu_qz = 0.f;
    float    gyr_x = 0.f, gyr_y = 0.f, gyr_z = 0.f;
};
#pragma pack(pop)

// ── helper: spawn ros2 joint controller ──────────────────────────────────────

static void StartJointController(const std::vector<std::string> &names)
{
    const char *ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
            throw std::runtime_error("Failed to create temporary parameter file");

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";
        for (const auto &name : names)
            tmp_file << "            - " << name << "\n";
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);
        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
            throw std::runtime_error("Failed to start joint controller");
        std::filesystem::remove(tmp_path);
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
}

// ── helper: quaternion to projected gravity ──────────────────────────────────

static void quatToProjectedGravity(float qw, float qx, float qy, float qz,
                                   float &pg_x, float &pg_y, float &pg_z)
{
    pg_x =  2.f * (qx * qz + qw * qy);
    pg_y =  2.f * (qy * qz - qw * qx);
    pg_z = -(qw * qw - qx * qx - qy * qy + qz * qz);
}

static void quatToRollPitch(float qw, float qx, float qy, float qz,
                            float &roll_deg, float &pitch_deg)
{
    float roll  = std::atan2(2.f * (qw * qx + qy * qz), 1.f - 2.f * (qx * qx + qy * qy));
    float pitch = std::asin(std::clamp(2.f * (qw * qy - qz * qx), -1.f, 1.f));
    roll_deg  = roll  * 180.f / M_PI;
    pitch_deg = pitch * 180.f / M_PI;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rl_mirror_sim_node");

    // ── Parse UDP port from CLI ─────────────────────────────────────────────
    int udp_port = 12345;
    // Skip ROS2 remapping args, look for a plain number
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (!arg.empty() && arg[0] != '-' && arg.find(":=") == std::string::npos)
        {
            udp_port = std::atoi(arg.c_str());
            if (udp_port > 0) break;
        }
    }
    RCLCPP_INFO(node->get_logger(), "UDP listen port: %d", udp_port);

    // ── Get robot_name from param_node (set by gazebo.launch.py) ────────────
    auto param_client = node->create_client<rcl_interfaces::srv::GetParameters>(
        "/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for param_node");
            return 1;
        }
        RCLCPP_WARN(node->get_logger(), "Waiting for param_node service...");
    }

    std::string robot_name = "Qmini";
    {
        auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        req->names = {"robot_name"};
        auto future = param_client->async_send_request(req);
        auto status = rclcpp::spin_until_future_complete(
            node->get_node_base_interface(), future, std::chrono::seconds(5));
        if (status == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future.get();
            if (!result->values.empty())
                robot_name = result->values[0].string_value;
        }
    }
    RCLCPP_INFO(node->get_logger(), "Robot: %s", robot_name.c_str());

    // ── Load YAML config ────────────────────────────────────────────────────
    // POLICY_DIR is defined at compile time
    std::string yaml_path;
    {
#ifdef POLICY_DIR
        yaml_path = std::string(POLICY_DIR) + "/" + robot_name + "/base.yaml";
#else
        yaml_path = "policy/" + robot_name + "/base.yaml";
#endif
    }
    RCLCPP_INFO(node->get_logger(), "Loading config: %s", yaml_path.c_str());

    YAML::Node root;
    try { root = YAML::LoadFile(yaml_path); }
    catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load %s", yaml_path.c_str());
        return 1;
    }

    YAML::Node cfg = root[robot_name];
    if (!cfg)
    {
        RCLCPP_ERROR(node->get_logger(), "No '%s' section in yaml", robot_name.c_str());
        return 1;
    }

    const int num_dofs = cfg["num_of_dofs"].as<int>(10);

    std::vector<int> joint_mapping(num_dofs);
    if (cfg["joint_mapping"])
        for (int i = 0; i < num_dofs; ++i)
            joint_mapping[i] = cfg["joint_mapping"][i].as<int>(i);

    std::vector<std::string> joint_names;
    if (cfg["joint_names"])
        for (int i = 0; i < num_dofs; ++i)
            joint_names.push_back(cfg["joint_names"][i].as<std::string>());

    std::vector<float> fixed_kp(num_dofs, 100.f), fixed_kd(num_dofs, 2.f);
    if (cfg["fixed_kp"])
        for (int i = 0; i < num_dofs; ++i)
            fixed_kp[i] = cfg["fixed_kp"][i].as<float>(100.f);
    if (cfg["fixed_kd"])
        for (int i = 0; i < num_dofs; ++i)
            fixed_kd[i] = cfg["fixed_kd"][i].as<float>(2.f);

    // ── Start Gazebo joint controller ───────────────────────────────────────
    std::string ros_namespace = node->get_namespace();
    try {
        StartJointController(joint_names);
    } catch (const std::exception &e) {
        RCLCPP_WARN(node->get_logger(), "Joint controller spawn: %s (may already be active, continuing)", e.what());
    }

    // ── ROS2 publisher for joint commands ────────────────────────────────────
    auto cmd_pub = node->create_publisher<robot_msgs::msg::RobotCommand>(
        ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // ── Gazebo IMU subscriber (for comparison display) ──────────────────────
    std::mutex imu_mutex;
    float sim_qw = 1.f, sim_qx = 0.f, sim_qy = 0.f, sim_qz = 0.f;
    float sim_gx = 0.f, sim_gy = 0.f, sim_gz = 0.f;

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(),
        [&](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lk(imu_mutex);
            sim_qw = msg->orientation.w;
            sim_qx = msg->orientation.x;
            sim_qy = msg->orientation.y;
            sim_qz = msg->orientation.z;
            sim_gx = msg->angular_velocity.x;
            sim_gy = msg->angular_velocity.y;
            sim_gz = msg->angular_velocity.z;
        });

    // ── Gazebo joint state subscriber (for comparison) ──────────────────────
    std::mutex state_mutex;
    std::vector<float> sim_joint_q(num_dofs, 0.f);

    auto state_sub = node->create_subscription<robot_msgs::msg::RobotState>(
        ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [&](const robot_msgs::msg::RobotState::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lk(state_mutex);
            for (int i = 0; i < num_dofs && i < (int)msg->motor_state.size(); ++i)
                sim_joint_q[joint_mapping[i]] = msg->motor_state[joint_mapping[i]].q;
        });

    // ── Create UDP receiver socket ──────────────────────────────────────────
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to create UDP socket");
        return 1;
    }

    // Set receive timeout (100ms) so we can check rclcpp::ok() periodically
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Allow reuse
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr{};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(udp_port);
    bind_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to bind UDP port %d: %s", udp_port, strerror(errno));
        close(sock);
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Listening on UDP port %d", udp_port);

    // ── Prepare command message ─────────────────────────────────────────────
    robot_msgs::msg::RobotCommand cmd_msg;
    cmd_msg.motor_command.resize(num_dofs);

    // ── Main loop ───────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
        "=== Joint Mirror Mode ===\n"
        "  Waiting for UDP data from rl_mirror_qmini...\n"
        "  The Gazebo robot will follow the real robot's joint angles.\n"
        "  IMU comparison (real vs sim) is printed at ~2 Hz.\n"
        "  Press Ctrl+C to quit.");

    MirrorPacket pkt;
    uint32_t last_seq = 0;
    int print_counter = 0;
    int timeout_counter = 0;
    bool receiving = false;

    // Spin ROS2 callbacks in a background thread
    std::thread spin_thread([&]()
    {
        rclcpp::spin(node);
    });

    while (rclcpp::ok())
    {
        ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0, nullptr, nullptr);

        if (n < (ssize_t)sizeof(pkt))
        {
            // Timeout or short packet
            if (!receiving)
            {
                if (++timeout_counter % 20 == 0)  // every ~2 sec
                    RCLCPP_WARN(node->get_logger(),
                        "No UDP data received yet. Ensure rl_mirror_qmini is running on Jetson.");
            }
            continue;
        }

        if (pkt.magic != MIRROR_MAGIC)
            continue;

        if (!receiving)
        {
            RCLCPP_INFO(node->get_logger(), "Receiving data from real robot!");
            receiving = true;
        }

        // ── Drive Gazebo joints ─────────────────────────────────────────
        for (int i = 0; i < num_dofs; ++i)
        {
            int mapped = joint_mapping[i];
            cmd_msg.motor_command[mapped].q   = pkt.q[i];
            cmd_msg.motor_command[mapped].dq  = 0.f;
            cmd_msg.motor_command[mapped].kp  = fixed_kp[i];
            cmd_msg.motor_command[mapped].kd  = fixed_kd[i];
            cmd_msg.motor_command[mapped].tau = 0.f;
        }
        cmd_pub->publish(cmd_msg);

        // ── Print comparison at ~2 Hz ───────────────────────────────────
        if (++print_counter >= 100)  // 100 packets @ 200Hz = 0.5s
        {
            print_counter = 0;

            // Real IMU
            float real_roll, real_pitch;
            quatToRollPitch(pkt.imu_qw, pkt.imu_qx, pkt.imu_qy, pkt.imu_qz,
                            real_roll, real_pitch);
            float real_pg_x, real_pg_y, real_pg_z;
            quatToProjectedGravity(pkt.imu_qw, pkt.imu_qx, pkt.imu_qy, pkt.imu_qz,
                                   real_pg_x, real_pg_y, real_pg_z);

            // Sim IMU
            float s_qw, s_qx, s_qy, s_qz, s_gx, s_gy, s_gz;
            {
                std::lock_guard<std::mutex> lk(imu_mutex);
                s_qw = sim_qw; s_qx = sim_qx; s_qy = sim_qy; s_qz = sim_qz;
                s_gx = sim_gx; s_gy = sim_gy; s_gz = sim_gz;
            }
            float sim_roll, sim_pitch;
            quatToRollPitch(s_qw, s_qx, s_qy, s_qz, sim_roll, sim_pitch);
            float sim_pg_x, sim_pg_y, sim_pg_z;
            quatToProjectedGravity(s_qw, s_qx, s_qy, s_qz, sim_pg_x, sim_pg_y, sim_pg_z);

            // Sim joint angles
            std::vector<float> sq;
            {
                std::lock_guard<std::mutex> lk(state_mutex);
                sq = sim_joint_q;
            }

            std::printf("\n╔══════════════════════════════════════════════════════════════════════╗\n");
            std::printf("║  Joint Mirror — seq=%u  (dropped: %u)                         ║\n",
                        pkt.seq, pkt.seq - last_seq - 1);
            std::printf("╠══════════════════════════════════════════════════════════════════════╣\n");
            std::printf("║  Joint  │  Real(rad)  │  Sim(rad)  │  Δ(rad)  │  Δ(deg)           ║\n");
            std::printf("║─────────┼─────────────┼────────────┼──────────┼───────────────────║\n");

            static const char *JNAMES[10] = {
                "L-HipYaw", "L-HipRol", "L-HipPit",
                "L-Knee  ", "L-Ankle ",
                "R-HipYaw", "R-HipRol", "R-HipPit",
                "R-Knee  ", "R-Ankle "
            };

            for (int i = 0; i < num_dofs; ++i)
            {
                float delta = pkt.q[i] - sq[i];
                std::printf("║  J%d %s │  %+7.3f    │  %+7.3f   │ %+7.3f  │ %+7.2f°          ║\n",
                            i, JNAMES[i], pkt.q[i], sq[i], delta, delta * 180.f / M_PI);
            }

            std::printf("╠══════════════════════════════════════════════════════════════════════╣\n");
            std::printf("║  IMU        │     Real         │     Sim          │     Δ           ║\n");
            std::printf("║─────────────┼──────────────────┼──────────────────┼─────────────────║\n");
            std::printf("║  Roll  (°)  │  %+8.2f        │  %+8.2f        │  %+8.2f        ║\n",
                        real_roll, sim_roll, real_roll - sim_roll);
            std::printf("║  Pitch (°)  │  %+8.2f        │  %+8.2f        │  %+8.2f        ║\n",
                        real_pitch, sim_pitch, real_pitch - sim_pitch);
            std::printf("║  pg_z       │  %+8.4f        │  %+8.4f        │  %+8.4f        ║\n",
                        real_pg_z, sim_pg_z, real_pg_z - sim_pg_z);
            std::printf("║  gyr_x      │  %+8.3f        │  %+8.3f        │  %+8.3f        ║\n",
                        pkt.gyr_x, s_gx, pkt.gyr_x - s_gx);
            std::printf("║  gyr_y      │  %+8.3f        │  %+8.3f        │  %+8.3f        ║\n",
                        pkt.gyr_y, s_gy, pkt.gyr_y - s_gy);
            std::printf("║  gyr_z      │  %+8.3f        │  %+8.3f        │  %+8.3f        ║\n",
                        pkt.gyr_z, s_gz, pkt.gyr_z - s_gz);
            std::printf("╚══════════════════════════════════════════════════════════════════════╝\n");
            std::fflush(stdout);

            last_seq = pkt.seq;
        }
    }

    close(sock);
    rclcpp::shutdown();
    if (spin_thread.joinable())
        spin_thread.join();

    return 0;
}
