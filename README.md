# rl_sar

Sim-to-real framework for reinforcement learning policy deployment on legged and humanoid robots.

## Supported Robots

| Robot | rname | Policy | Gazebo | Real |
|-------|-------|--------|:------:|:----:|
| NIO ATOM01 | `ATOM01` | nio_lab | ✅ | ✅ |
| NIO Qmini | `Qmini` | nio_lab | ✅ | ✅ |
| Unitree G1 | `g1` | nio_lab / whole_body_tracking | ✅ | ✅ |
| Unitree Go2W | `go2w` | robot_lab | ✅ | ✅ |

## Quick Start

### 1. Dependencies

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
    libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# ROS2 simulation dependencies
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

### 2. Build

```bash
./build.sh              # Full build (colcon + CMake)
./build.sh pkg1 pkg2    # Build specific packages
./build.sh -m           # CMake only (hardware deployment, no ROS)
./build.sh -c           # Clean build artifacts
```

### 3. Gazebo Simulation

```bash
# Terminal 1: launch simulation
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>

# Terminal 2: launch controller
source install/setup.bash
ros2 run rl_sar rl_sim
```

If Gazebo has no models on first launch: `git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models`

### 4. Real Robot

```bash
# ROS2 (Unitree and other Ethernet-based robots require a network interface)
source install/setup.bash
ros2 run rl_sar rl_real_<ROBOT> <NETWORK_INTERFACE>

# CMake (no ROS)
# Unitree series (Ethernet communication — identify the interface first)
ip link show                              # list interfaces
./cmake_build/bin/rl_real_<ROBOT> <NETWORK_INTERFACE>

# Qmini (serial communication — no network interface argument)
sudo ./cmake_build/bin/rl_real_qmini
```

## Controls

| Gamepad | Keyboard | Function |
|---------|----------|----------|
| A | `0` | Stand up |
| B | `9` | Lie down |
| X | `N` | Navigation mode |
| RB+Y | `R` | Reset Gazebo |
| RB+X | `Enter` | Gazebo pause/resume |
| LB+X | `P` | Passive mode |
| RB+↑ / Y | `1` | Locomotion mode |
| LY / LX | `W/S` `A/D` | Forward-back / lateral |
| RX | `Q/E` | Yaw rotation |
| — | `Space` | Zero velocity |

**Gamepad diagnostics** — if button mapping differs, use diagnostic scripts then update source code:

```bash
python3 scripts/test_joystick.py          # Real robot (reads /dev/input/js0)
python3 scripts/test_joy_ros.py           # Gazebo (reads /joy topic)
```

## Jetson On-board Deployment (Qmini Example)

### Setup

```bash
# System dependencies
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
    libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev

# LibTorch (auto-detects JetPack version)
bash scripts/install_pytorch_jetson.sh
```

ONNX Runtime is automatically disabled on Jetson; only LibTorch is used for inference.

### Network Connection

The Jetson is connected to the dev PC via wired Ethernet or hotspot, defaulting to `172.20.10.3`.

```bash
# Find the network interface connected to the Jetson
ip link show

# Example output (find the interface connected to Jetson):
# 2: enP8p1s0: <BROADCAST,MULTICAST,UP> ...
# 3: wlp2s0:   <BROADCAST,MULTICAST,UP> ...

# Verify connectivity
ping 172.20.10.3
```

### Build and Run

```bash
./build.sh -m                          # Build
sudo ./cmake_build/bin/rl_real_qmini   # Run (Qmini uses serial — no interface argument)
```

Workflow: A (stand) → Y (locomotion) → joystick control → LB+X (passive) → B (lie down).

### Code Sync

After modifying code on the dev machine, sync to Jetson and build remotely:

```bash
ssh-copy-id qmini@172.20.10.3          # First time: passwordless SSH (confirm interface first)
./scripts/sync_to_jetson.sh             # Sync + build
./scripts/sync_to_jetson.sh --no-build  # Sync only
```

Build artifacts: `cmake_build/bin/{rl_real_qmini, rl_calib_qmini, rl_mirror_qmini}`

### Encoder Calibration

Required after replacing motors or mechanical parts. Calibration values are stored in `encoder_offsets` in `policy/Qmini/base.yaml`.

```bash
sudo ./cmake_build/bin/rl_calib_qmini
```

**Mode 1 — Encoder Zero-offset Calibration**

1. Place the robot in standard standing pose (matching `default_dof_pos`)
2. Run the tool, select `1`
3. The tool connects motors in zero-torque mode, displays joint angles in real time
4. Press Enter to capture, enter `y` to write to `base.yaml`

Principle: `new_offset[i] = old_offset[i] + (current_q[i] − default_dof_pos[i])`

**Mode 2 — Joint Limit Scan**

Measures mechanical limits for each joint, writes to `dof_pos_limits_min` / `dof_pos_limits_max`.

1. Select `2`, all joints are in zero-torque (free to move)
2. Enter joint index `0`–`9` to start min/max tracking
3. Move the joint through its full range, press Enter to confirm
4. Enter `s` to save

| Command | Description |
|---------|-------------|
| `0`–`9` | Select joint |
| `p` | Print current limit table |
| `s` | Save to `base.yaml` |
| `q` | Quit without saving |
| `c` | Cancel current joint recording |

Sync calibration back to dev machine and update the training URDF:

```bash
# Pull calibrated limits from Jetson
scp qmini@172.20.10.3:/home/qmini/sim2real/rl_sar/policy/Qmini/base.yaml policy/Qmini/base.yaml

# Write the limits from base.yaml into the URDF (for retraining)
python3 scripts/sync_limits_to_urdf.py            # update URDF
python3 scripts/sync_limits_to_urdf.py --dry-run  # preview changes only
python3 scripts/sync_limits_to_urdf.py --margin 2 # shrink limits inward by 2°
```

The script validates min < max, checks left-right leg symmetry, and backs up the original URDF.

### Joint Mirror Test

Verifies joint direction and sign consistency between real hardware and URDF simulation. The Jetson reads joints and IMU in zero-torque mode and streams data via UDP; the PC drives a fixed-base Gazebo model to follow the real robot.

```bash
# First, confirm the PC IP on the interface connected to Jetson
ip link show                             # find the connected interface
ip addr show <INTERFACE>                 # check PC IP (typically 172.20.10.2)

# PC Terminal 1: launch Gazebo (robot fixed in air)
source install/setup.bash
ros2 launch rl_sar mirror.launch.py rname:=Qmini

# PC Terminal 2: launch mirror receiver
source install/setup.bash
ros2 run rl_sar rl_mirror_sim

# Jetson: launch mirror sender (use PC IP found above)
sudo ./cmake_build/bin/rl_mirror_qmini <PC_IP>
# e.g.: sudo ./cmake_build/bin/rl_mirror_qmini 172.20.10.2
```

The PC prints a real vs. simulation joint angle comparison table for per-joint direction verification.

### Log Analysis

`rl_real_qmini` automatically generates CSV logs (joint angles, IMU, FSM state, etc.) under `policy/Qmini/`. Visualize with:

```bash
python3 scripts/plot_rl_log.py                           # Auto-load latest log
python3 scripts/plot_rl_log.py policy/Qmini/rl_log_*.csv # Specify file
```

## Configuration

Core configuration for each robot is in `policy/<ROBOT>/base.yaml`. Key fields:

| Field | Description |
|-------|-------------|
| `motor_sign` | Per-motor direction sign (+1/-1), maps hardware encoder to URDF frame |
| `encoder_offsets` | Encoder zero-offsets (written by calibration tool) |
| `default_dof_pos` | Standard standing joint angles |
| `dof_pos_limits_min/max` | Joint position limits (URDF frame) |
| `fixed_kp` / `fixed_kd` | Joint PD control gains |
| `joint_mapping` | URDF joint to hardware motor index mapping |

Policy directory structure:

```
policy/<ROBOT>/
    base.yaml
    <CONFIG>/
        config.yaml
        policy.onnx
```

## Adding a Custom Robot

```
src/rl_sar_zoo/<ROBOT>_description/
    CMakeLists.txt
    package.ros2.xml
    xacro/robot.xacro
    xacro/gazebo.xacro
    config/robot_control.yaml
    config/robot_control_ros2.yaml

policy/<ROBOT>/
    base.yaml
    <CONFIG>/config.yaml
    <CONFIG>/<POLICY>.onnx

src/rl_sar/fsm_robot/
    fsm_<ROBOT>.hpp
    fsm_all.hpp        ← register new robot

src/rl_sar/src/
    rl_real_<ROBOT>.cpp
```

Refer to the `go2w` directory structure for guidance.

> **Caution** — All risks and consequences arising from the use of this code are borne by the user. Ensure adequate safety measures before operation.
