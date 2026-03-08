/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_qmini.hpp"

RL_Real::RL_Real(int argc, char **argv)
{
#if defined(USE_ROS2) && defined(USE_ROS)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); });
#endif

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name   = "Qmini";
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // init joystick (evdev /dev/input/js0)
    this->initJoystick();

    std::cout << std::endl
              << LOGGER::NOTE << "=========================================" << std::endl
              << LOGGER::NOTE << " SAFE MODE: Motors are in ZERO-TORQUE.   " << std::endl
              << LOGGER::NOTE << " Place the robot on the ground first!     " << std::endl
              << LOGGER::NOTE << " Press '0' (keyboard) or 'A' (gamepad)   " << std::endl
              << LOGGER::NOTE << " to ARM motors and enter GetUp.           " << std::endl
              << LOGGER::NOTE << "=========================================" << std::endl;

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control  = std::make_shared<LoopFunc>("loop_control",  this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl       = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();
    if (this->js_fd_ >= 0)
    {
        this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.02, std::bind(&RL_Real::readJoystick, this));
        this->loop_joystick->start();
    }

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    if (this->loop_joystick) this->loop_joystick->shutdown();
    if (this->js_fd_ >= 0) close(this->js_fd_);
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<float> *state)
{
    // Read motor feedback from serial
    auto ms = this->motor_ctrl.getState();
    const auto &jmap = this->params.Get<std::vector<int>>("joint_mapping");
    const int ndof   = this->params.Get<int>("num_of_dofs");

    for (int i = 0; i < ndof; ++i)
    {
        int hw_idx = jmap[i];
        state->motor_state.q[i]       = ms[hw_idx].q;
        state->motor_state.dq[i]      = ms[hw_idx].dq;
        state->motor_state.tau_est[i] = ms[hw_idx].tau;
    }

    // IMU (CP2102 on /dev/ttyUSB4)
    auto imu = this->imu_.get();
    if (imu.valid)
    {
        state->imu.quaternion[0] = imu.qw;
        state->imu.quaternion[1] = imu.qx;
        state->imu.quaternion[2] = imu.qy;
        state->imu.quaternion[3] = imu.qz;
        state->imu.gyroscope[0]  = imu.gyro_x;
        state->imu.gyroscope[1]  = imu.gyro_y;
        state->imu.gyroscope[2]  = imu.gyro_z;
    }
    else
    {
        // IMU not ready yet — stay upright
        state->imu.quaternion[0] = 1.f;
        state->imu.quaternion[1] = 0.f;
        state->imu.quaternion[2] = 0.f;
        state->imu.quaternion[3] = 0.f;
        state->imu.gyroscope[0]  = 0.f;
        state->imu.gyroscope[1]  = 0.f;
        state->imu.gyroscope[2]  = 0.f;
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    const auto &jmap = this->params.Get<std::vector<int>>("joint_mapping");
    const int ndof   = this->params.Get<int>("num_of_dofs");

    for (int i = 0; i < ndof; ++i)
    {
        int hw_idx = jmap[i];
        if (this->motors_armed_)
        {
            this->motor_cmd_[hw_idx].q   = command->motor_command.q[i];
            this->motor_cmd_[hw_idx].dq  = command->motor_command.dq[i];
            this->motor_cmd_[hw_idx].kp  = command->motor_command.kp[i];
            this->motor_cmd_[hw_idx].kd  = command->motor_command.kd[i];
            this->motor_cmd_[hw_idx].tau = command->motor_command.tau[i];
        }
        else
        {
            // Safety gate: motors not armed yet — send zero torque (back-drivable)
            this->motor_cmd_[hw_idx].q   = 0.f;
            this->motor_cmd_[hw_idx].dq  = 0.f;
            this->motor_cmd_[hw_idx].kp  = 0.f;
            this->motor_cmd_[hw_idx].kd  = 0.f;
            this->motor_cmd_[hw_idx].tau = 0.f;
        }
    }
    this->motor_ctrl.setCommand(this->motor_cmd_);
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    // Arm motors when FSM leaves Passive (i.e., user pressed 'A' / Gamepad::A)
    if (!this->motors_armed_ && this->fsm.current_state_ &&
        this->fsm.current_state_->GetStateName() != "RLFSMStatePassive")
    {
        this->motor_ctrl.enableMotors();
        this->motors_armed_ = true;
        std::cout << std::endl << LOGGER::NOTE
                  << "Motors ARMED — FSM entered "
                  << this->fsm.current_state_->GetStateName() << std::endl;
    }

    // In safe mode (not armed): print joint positions + IMU at ~1 Hz so the
    // user can verify sensors are working before pressing 'A'.
    if (!this->motors_armed_)
    {
        static int diag_counter = 0;
        if (++diag_counter >= 200)   // 200 * 5ms = 1s
        {
            diag_counter = 0;
            const auto &q  = this->robot_state.motor_state.q;
            const auto &qd = this->robot_state.motor_state.dq;
            const auto &qt = this->robot_state.imu.quaternion;
            const auto &gyr = this->robot_state.imu.gyroscope;
            // Joint positions (hardware order 0-9)
            std::cout << "\r\033[K" << std::flush;
            std::printf("[DIAG] q(rad): ");
            for (int i = 0; i < (int)q.size(); ++i)
                std::printf("%6.3f ", q[i]);
            std::printf("\n");
            // IMU quaternion + gyro
            std::printf("[DIAG] IMU qw=%6.3f qx=%6.3f qy=%6.3f qz=%6.3f  gyr: %6.3f %6.3f %6.3f\n",
                qt[0], qt[1], qt[2], qt[3], gyr[0], gyr[1], gyr[2]);
            std::fflush(stdout);
        }
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel  = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos   = this->robot_state.motor_state.q;
        this->obs.dof_vel   = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())  output_dof_pos_queue.push(this->output_dof_pos);
        if (!this->output_dof_vel.empty())  output_dof_vel_queue.push(this->output_dof_vel);
        if (!this->output_dof_tau.empty())  output_dof_tau_queue.push(this->output_dof_tau);

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    auto ms = this->motor_ctrl.getState();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(ms[i].q);
        this->plot_target_joint_pos[i].push_back(this->motor_cmd_[i].q);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos",   this->plot_t, this->plot_real_joint_pos[i],   "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}

void RL_Real::initJoystick()
{
    const char *js_path = "/dev/input/js0";
    this->js_fd_ = open(js_path, O_RDONLY | O_NONBLOCK);
    if (this->js_fd_ < 0)
        std::cout << LOGGER::WARNING << "Joystick not found at " << js_path
                  << ", gamepad disabled (keyboard still works)" << std::endl;
    else
        std::cout << LOGGER::INFO << "Joystick opened: " << js_path << std::endl;
}

void RL_Real::readJoystick()
{
    if (this->js_fd_ < 0) return;

    struct js_event e;
    static std::array<float, 16> axes_{};
    static std::array<bool, 32>  btns_{};
    // Debug log file (always open/close to avoid buffering)
    auto js_log = [](const char *msg) {
        FILE *f = fopen("/tmp/js_debug.txt", "a");
        if (f) { fputs(msg, f); fclose(f); }
    };

    while (read(this->js_fd_, &e, sizeof(e)) > 0)
    {
        // Skip synthetic init events
        if (e.type & JS_EVENT_INIT) continue;

        if (e.type & JS_EVENT_BUTTON)
        {
            int id = e.number;
            bool pressed = e.value != 0;
            if (id < (int)btns_.size()) btns_[id] = pressed;

            // Debug: write to file (unbuffered, SSH-safe)
            {
                char buf[128];
                snprintf(buf, sizeof(buf), "[JS] BUTTON id=%d val=%d  btns[4]=%d btns[5]=%d\n",
                    id, (int)pressed, (int)btns_[4], (int)btns_[5]);
                js_log(buf);
            }

            // Qmini gamepad layout (from official joystick.py):
            // A=0, B=1, X=3, Y=4, L1=6, R1=7, L2=8, R2=9, SELECT=10, START=11
            // DPad is reported as hat (axis 6/7), not buttons
            auto setBtn = [&](int bid, Input::Gamepad g){ if(id==bid && pressed) this->control.SetGamepad(g); };
            setBtn(0,  Input::Gamepad::A);
            setBtn(1,  Input::Gamepad::B);
            setBtn(3,  Input::Gamepad::X);
            setBtn(4,  Input::Gamepad::Y);
            setBtn(6,  Input::Gamepad::LB);
            setBtn(7,  Input::Gamepad::RB);
            setBtn(10, Input::Gamepad::LStick);   // SELECT
            setBtn(11, Input::Gamepad::RStick);   // START

            // Combo: L1(6) + face buttons
            if (btns_[6]) {
                if (id==0 && pressed) this->control.SetGamepad(Input::Gamepad::LB_A);
                if (id==1 && pressed) this->control.SetGamepad(Input::Gamepad::LB_B);
                if (id==3 && pressed) this->control.SetGamepad(Input::Gamepad::LB_X);
                if (id==4 && pressed) this->control.SetGamepad(Input::Gamepad::LB_Y);
            }
            // Combo: R1(7) + face buttons
            if (btns_[7]) {
                if (id==0 && pressed) this->control.SetGamepad(Input::Gamepad::RB_A);
                if (id==1 && pressed) this->control.SetGamepad(Input::Gamepad::RB_B);
                if (id==3 && pressed) this->control.SetGamepad(Input::Gamepad::RB_X);
                if (id==4 && pressed) this->control.SetGamepad(Input::Gamepad::RB_Y);
            }
            if (btns_[6] && btns_[7] && pressed) this->control.SetGamepad(Input::Gamepad::LB_RB);
        }
        else if (e.type & JS_EVENT_AXIS)
        {
            int id = e.number;
            float val = e.value / 32767.f;
            if (id < (int)axes_.size()) axes_[id] = val;

            // Axis layout (Qmini gamepad, from official joystick.py):
            // 0=LX, 1=LY, 2=RX, 3=RY, 4=L2(trigger), 5=R2(trigger), 6=DPadX, 7=DPadY
            this->control.x   = -axes_[1];  // LY up=forward
            this->control.y   = -axes_[0];  // LX left=strafe
            this->control.yaw = -axes_[2];  // RX left=yaw left

            // DPad as axis (hat): axis6=DPadX, axis7=DPadY
            if (id == 6) {
                if (val >  0.5f) { this->control.SetGamepad(Input::Gamepad::DPadRight);
                    if (btns_[7]) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
                    if (btns_[6]) this->control.SetGamepad(Input::Gamepad::LB_DPadRight); }
                if (val < -0.5f) { this->control.SetGamepad(Input::Gamepad::DPadLeft);
                    if (btns_[7]) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
                    if (btns_[6]) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft); }
            }
            if (id == 7) {
                if (val >  0.5f) { this->control.SetGamepad(Input::Gamepad::DPadDown);
                    if (btns_[7]) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
                    if (btns_[6]) this->control.SetGamepad(Input::Gamepad::LB_DPadDown); }
                if (val < -0.5f) { this->control.SetGamepad(Input::Gamepad::DPadUp);
                    if (btns_[7]) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
                    if (btns_[6]) this->control.SetGamepad(Input::Gamepad::LB_DPadUp); }
            }
        }
    }

    // Re-assert held button state every cycle so ClearInput() between
    // joystick polls does not swallow a briefly-held button press.
    // Priority: combos first, then single buttons.
    if      (btns_[6] && btns_[7]) this->control.SetGamepad(Input::Gamepad::LB_RB);
    else if (btns_[6] && btns_[0]) this->control.SetGamepad(Input::Gamepad::LB_A);
    else if (btns_[6] && btns_[1]) this->control.SetGamepad(Input::Gamepad::LB_B);
    else if (btns_[6] && btns_[3]) this->control.SetGamepad(Input::Gamepad::LB_X);
    else if (btns_[6] && btns_[4]) this->control.SetGamepad(Input::Gamepad::LB_Y);
    else if (btns_[7] && btns_[0]) this->control.SetGamepad(Input::Gamepad::RB_A);
    else if (btns_[7] && btns_[1]) this->control.SetGamepad(Input::Gamepad::RB_B);
    else if (btns_[7] && btns_[3]) this->control.SetGamepad(Input::Gamepad::RB_X);
    else if (btns_[7] && btns_[4]) this->control.SetGamepad(Input::Gamepad::RB_Y);
    else if (btns_[0]) this->control.SetGamepad(Input::Gamepad::A);
    else if (btns_[1]) this->control.SetGamepad(Input::Gamepad::B);
    else if (btns_[3]) this->control.SetGamepad(Input::Gamepad::X);
    else if (btns_[4]) this->control.SetGamepad(Input::Gamepad::Y);
    else if (btns_[6]) this->control.SetGamepad(Input::Gamepad::LB);
    else if (btns_[7]) this->control.SetGamepad(Input::Gamepad::RB);
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}
#endif

int main(int argc, char **argv)
{
    // networkInterface argument no longer needed (serial port, not DDS)
    // kept for CLI compatibility but ignored
#if defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar(argc, argv);
    while (1) { sleep(10); }
#endif

    return 0;
}
