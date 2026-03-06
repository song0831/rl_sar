/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_QMINI_HPP
#define RL_REAL_QMINI_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_qmini.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <csignal>
#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>

#if defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

static const std::string QMINI_CMD_TOPIC   = "rt/lowcmd";
static const std::string QMINI_STATE_TOPIC = "rt/lowstate";
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

// ---------- joystick raw-data structs (same as G1) ----------
typedef union
{
    struct
    {
        uint8_t R1     : 1;
        uint8_t L1     : 1;
        uint8_t start  : 1;
        uint8_t select : 1;
        uint8_t R2     : 1;
        uint8_t L2     : 1;
        uint8_t F1     : 1;
        uint8_t F2     : 1;
        uint8_t A      : 1;
        uint8_t B      : 1;
        uint8_t X      : 1;
        uint8_t Y      : 1;
        uint8_t up     : 1;
        uint8_t right  : 1;
        uint8_t down   : 1;
        uint8_t left   : 1;
    } components;
    uint16_t value;
} QminiKeySwitchUnion;

typedef struct
{
    uint8_t head[2];
    QminiKeySwitchUnion btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;
    uint8_t idle[16];
} QminiRockerBtnDataStruct;

typedef union
{
    QminiRockerBtnDataStruct RF_RX;
    uint8_t buff[40];
} QMINI_REMOTE_DATA_RX;

class QminiButton
{
public:
    QminiButton() {}

    void update(bool state)
    {
        on_press   = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed    = state;
    }

    bool pressed    = false;
    bool on_press   = false;
    bool on_release = false;
};

class QminiGamepad
{
public:
    QminiGamepad() {}

    void update(QminiRockerBtnDataStruct &key_data)
    {
        lx = lx * (1 - smooth) + (std::fabs(key_data.lx) < dead_zone ? 0.0f : key_data.lx) * smooth;
        rx = rx * (1 - smooth) + (std::fabs(key_data.rx) < dead_zone ? 0.0f : key_data.rx) * smooth;
        ry = ry * (1 - smooth) + (std::fabs(key_data.ry) < dead_zone ? 0.0f : key_data.ry) * smooth;
        l2 = l2 * (1 - smooth) + (std::fabs(key_data.L2) < dead_zone ? 0.0f : key_data.L2) * smooth;
        ly = ly * (1 - smooth) + (std::fabs(key_data.ly) < dead_zone ? 0.0f : key_data.ly) * smooth;

        R1.update(key_data.btn.components.R1);
        L1.update(key_data.btn.components.L1);
        start.update(key_data.btn.components.start);
        select.update(key_data.btn.components.select);
        R2.update(key_data.btn.components.R2);
        L2.update(key_data.btn.components.L2);
        F1.update(key_data.btn.components.F1);
        F2.update(key_data.btn.components.F2);
        A.update(key_data.btn.components.A);
        B.update(key_data.btn.components.B);
        X.update(key_data.btn.components.X);
        Y.update(key_data.btn.components.Y);
        up.update(key_data.btn.components.up);
        right.update(key_data.btn.components.right);
        down.update(key_data.btn.components.down);
        left.update(key_data.btn.components.left);
    }

    float lx = 0.f;
    float rx = 0.f;
    float ry = 0.f;
    float l2 = 0.f;
    float ly = 0.f;

    float smooth    = 0.03f;
    float dead_zone = 0.01f;

    QminiButton R1;
    QminiButton L1;
    QminiButton start;
    QminiButton select;
    QminiButton R2;
    QminiButton L2;
    QminiButton F1;
    QminiButton F2;
    QminiButton A;
    QminiButton B;
    QminiButton X;
    QminiButton Y;
    QminiButton up;
    QminiButton right;
    QminiButton down;
    QminiButton left;
};

enum class QminiMode {
    PR = 0,  // Series Control for Pitch/Roll Joints
    AB = 1   // Parallel Control for A/B Joints
};

// Qmini joint index (10 DOF: 5 per leg)
enum QminiJointIndex {
    LL_Joint1 = 0,
    LL_Joint2 = 1,
    LL_Joint3 = 2,
    LL_Joint4 = 3,
    LL_Joint5 = 4,
    RL_Joint1 = 5,
    RL_Joint2 = 6,
    RL_Joint3 = 7,
    RL_Joint4 = 8,
    RL_Joint5 = 9,
};

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();

#if defined(USE_ROS2) && defined(USE_ROS)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // unitree hg interface
    void InitLowCmd();
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void LowStateHandler(const void *message);
    LowCmd_   unitree_low_command;
    LowState_ unitree_low_state;
    QminiMode mode_pr;
    uint8_t   mode_machine;
    QminiGamepad    gamepad;
    QMINI_REMOTE_DATA_RX remote_data_rx;
    ChannelPublisherPtr<LowCmd_>   lowcmd_publisher;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber;

    // others
    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;

#if defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

#endif // RL_REAL_QMINI_HPP
