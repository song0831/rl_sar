# rl_sar

强化学习策略的仿真验证与实物部署框架，支持机器人与遥控车辆。| [English](README.md)

---

## 支持列表

| 机器人 | rname | Policy | Gazebo | Real |
|--------|-------|--------|--------|------|
| NIO ATOM01 | `ATOM01` | nio_lab | 是 | 是 |
| NIO Qmini | `Qmini` | nio_lab | 是 | 是 |
| MJX Hyper Go 7303 / ET6 | `MJX_ET6` | nio_lab | 否 | 是 |
| Unitree G1 | `g1` | nio_lab / whole_body_tracking | 是 | 是 |
| Unitree Go2W | `go2w` | robot_lab | 是 | 是 |

---

## MJX ET6 遥控车

仓库包含 MJX Hyper Go 7303 的 Jetson ONNX 推理目标、两路 PWM 输出、H30
IMU 与四轮转速观测，以及完全由手柄切换的遥控/策略运行框架。安装和安全操作见
[MJX ET6 Jetson 部署说明](docs/MJX_ET6_DEPLOYMENT.md)。

---

## 快速开始

### 1. 依赖安装

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \n    libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev
```

ROS2 仿真额外依赖：

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard \n    ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \n    ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher \n    ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control \n    ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

### 2. 编译

```bash
./build.sh              # 编译全部（colcon + CMake）
./build.sh pkg1 pkg2    # 编译指定包
./build.sh -m           # 仅 CMake（硬件部署，不依赖 ROS）
./build.sh -c           # 清理构建产物
```

### 3. Gazebo 仿真

终端 1 — 启动仿真环境：

```bash
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>
```

终端 2 — 启动控制程序：

```bash
source install/setup.bash
ros2 run rl_sar rl_sim
```

首次启动 Gazebo 若无模型，执行：

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

### 4. 真实机器人

ROS2 模式（Unitree 等以太网机器人需要指定网卡）：

```bash
source install/setup.bash
ros2 run rl_sar rl_real_<ROBOT> <NETWORK_INTERFACE>

# Go2W：添加 'wheel' 参数以启用轮式模式
ros2 run rl_sar rl_real_go2 <NETWORK_INTERFACE> wheel
```

CMake 模式（无 ROS）：

```bash
ip link show                                              # 确认网卡名
./cmake_build/bin/rl_real_<ROBOT> <NETWORK_INTERFACE>

# Go2W
./cmake_build/bin/rl_real_go2 <NETWORK_INTERFACE> wheel

# Qmini（串口通信，不需要网卡参数）
sudo ./cmake_build/bin/rl_real_qmini
```

---

## 控制方式

| 手柄 | 键盘 | 功能 |
|------|------|------|
| A | `0` | 站立 |
| B | `9` | 趴下 |
| X | `N` | 导航模式 |
| RB+Y | `R` | 重置 Gazebo |
| RB+X | `Enter` | Gazebo 暂停/继续 |
| LB+X | `P` | Passive 模式 |
| RB+↑ / Y | `1` | Locomotion 模式 |
| LY / LX | `W/S` `A/D` | 前后 / 左右 |
| RX | `Q/E` | Yaw 旋转 |
| — | `Space` | 速度清零 |

若按键映射不符，使用诊断脚本确认 id 后修改对应源码：

```bash
python3 scripts/test_joystick.py    # 真实机器人（读取 /dev/input/js0）
python3 scripts/test_joy_ros.py     # Gazebo 仿真（读取 /joy 话题）
```

---

## Jetson 机载部署（以 Qmini 为例）

### 环境准备

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \n    libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev

# LibTorch（脚本自动检测 JetPack 版本）
bash scripts/install_pytorch_jetson.sh
```

Jetson 平台自动禁用 ONNX Runtime，仅使用 LibTorch 推理。

### 网络连接

Jetson 通过有线以太网或热点与 PC 相连，默认 IP 为 `172.20.10.3`。

```bash
ip link show        # 找到连接 Jetson 的网卡名
ping 172.20.10.3    # 确认连通性
```

### 编译与运行

```bash
./build.sh -m                          # 编译
sudo ./cmake_build/bin/rl_real_qmini   # 运行
```

操作流程：A 站起 → Y 进入 Locomotion → 摇杆控制移动 → LB+X 退出 → B 趴下。

### 代码同步

```bash
ssh-copy-id qmini@172.20.10.3          # 首次：配置免密登录
./scripts/sync_to_jetson.sh             # 同步 + 编译
./scripts/sync_to_jetson.sh --no-build  # 仅同步
```

编译产物：`cmake_build/bin/{rl_real_qmini, rl_calib_qmini, rl_mirror_qmini}`

### 编码器标定

更换电机或机械零件后需重新标定。标定值存储在 `policy/Qmini/base.yaml` 的 `encoder_offsets` 字段。

```bash
sudo ./cmake_build/bin/rl_calib_qmini
```

**模式 1 — 编码器零点标定**

1. 将机器人摆放至标准站立姿态（与 `default_dof_pos` 一致）
2. 运行标定工具，选择 `1`
3. 工具以零力矩模式连接电机，实时显示关节角度
4. 确认站姿后按 Enter 采集，输入 `y` 写入 `base.yaml`

原理：`new_offset[i] = old_offset[i] + (current_q[i] - default_dof_pos[i])`

**模式 2 — 关节限位扫描**

测量每个关节的真实机械限位，写入 `dof_pos_limits_min` / `dof_pos_limits_max`。

1. 选择 `2`，所有关节处于零力矩可自由移动
2. 输入关节编号 `0`–`9`，开始追踪 min/max
3. 将关节在全范围内来回移动，按 Enter 确认
4. 输入 `s` 保存

| 命令 | 说明 |
|------|------|
| `0`–`9` | 选择关节 |
| `p` | 打印当前限位表 |
| `s` | 保存到 `base.yaml` |
| `q` | 不保存退出 |
| `c` | 取消当前关节录制 |

标定完成后同步回本机：

```bash
scp qmini@172.20.10.3:/home/qmini/sim2real/rl_sar/policy/Qmini/base.yaml policy/Qmini/base.yaml
```

### 关节镜像测试

验证实物与仿真的 URDF 关节方向、符号是否一致。Jetson 端以零力矩读取关节和 IMU，通过 UDP 发送到 PC；PC 端驱动 Gazebo 中固定悬空的机器人跟随实物运动。

PC 端 — 确认 IP：

```bash
ip link show                   # 找到连接 Jetson 的网卡名
ip addr show <网卡名>          # 查看 PC IP（通常为 172.20.10.2）
```

PC 终端 1 — 启动 Gazebo：

```bash
source install/setup.bash
ros2 launch rl_sar mirror.launch.py rname:=Qmini
```

PC 终端 2 — 启动镜像接收：

```bash
source install/setup.bash
ros2 run rl_sar rl_mirror_sim
```

Jetson — 启动镜像发送：

```bash
sudo ./cmake_build/bin/rl_mirror_qmini <PC_IP>
```

PC 端打印实物与仿真的关节角度对比表，用于逐关节确认方向一致性。

---

## 配置说明

核心配置位于 `policy/<ROBOT>/base.yaml`，关键字段：

| 字段 | 说明 |
|------|------|
| `motor_sign` | 每个电机的方向符号（+1/-1），将硬件编码器映射到 URDF 坐标系 |
| `encoder_offsets` | 编码器零点偏移量（标定工具自动写入） |
| `default_dof_pos` | 标准站立姿态关节角度 |
| `dof_pos_limits_min/max` | 关节位置限位（URDF 坐标系） |
| `fixed_kp` / `fixed_kd` | 关节 PD 控制增益 |
| `joint_mapping` | URDF 关节到硬件电机的索引映射 |

策略文件目录结构：

```
policy/<ROBOT>/
    base.yaml
    <CONFIG>/
        config.yaml
        policy.onnx  # 或 policy.pt
```

---

> 使用本代码产生的所有风险及后果由使用者自行承担，操作前请确保已采取充分安全防护措施。
