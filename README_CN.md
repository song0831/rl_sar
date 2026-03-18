# rl_sar

机器人强化学习策略的仿真验证与实物部署框架，支持轮足与人形机器人。

## 支持列表

| 机器人 | rname | Policy | Gazebo | Real |
|--------|-------|--------|:------:|:----:|
| NIO ATOM01 | `ATOM01` | nio_lab | ✅ | ✅ |
| NIO Qmini | `Qmini` | nio_lab | ✅ | ✅ |
| Unitree G1 | `g1` | nio_lab / whole_body_tracking | ✅ | ✅ |
| Unitree Go2W | `go2w` | robot_lab | ✅ | ✅ |

## 快速开始

### 1. 依赖安装

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
    libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# ROS2 仿真依赖
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

### 2. 编译

```bash
./build.sh              # 编译全部（colcon + CMake）
./build.sh pkg1 pkg2    # 编译指定包
./build.sh -m           # 仅 CMake（硬件部署，不依赖 ROS）
./build.sh -c           # 清理构建产物
```

### 3. Gazebo 仿真

```bash
# 终端 1：启动仿真环境
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>

# 终端 2：启动控制程序
source install/setup.bash
ros2 run rl_sar rl_sim
```

首次启动 Gazebo 若无模型：`git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models`

### 4. 真实机器人

```bash
# ROS2 模式（Unitree 等以太网机器人需要指定网卡）
source install/setup.bash
ros2 run rl_sar rl_real_<ROBOT> <NETWORK_INTERFACE>

# CMake 模式（无 ROS）
# Unitree 系列（以太网通信，需要网卡名）
ip link show                              # 确认网卡名
./cmake_build/bin/rl_real_<ROBOT> <NETWORK_INTERFACE>

# Qmini（串口通信，不需要网卡参数）
./cmake_build/bin/rl_real_qmini
```

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

**手柄诊断** — 若按键映射不符，使用诊断脚本确认 id 后修改对应源码：

```bash
python3 scripts/test_joystick.py          # 真实机器人（读取 /dev/input/js0）
python3 scripts/test_joy_ros.py           # Gazebo 仿真（读取 /joy 话题）
```

## Jetson 机载部署（以 Qmini 为例）

### 环境准备

```bash
# 系统依赖
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
    libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev

# LibTorch（脚本自动检测 JetPack 版本）
bash scripts/install_pytorch_jetson.sh
```

Jetson 平台自动禁用 ONNX Runtime，仅使用 LibTorch 推理。

### 网络连接

Jetson 通过有线以太网或热点与 PC 相连，默认 IP 为 `172.20.10.3`。

```bash
# 确认 PC 端与 Jetson 互通的网卡名
ip link show

# 示例输出（找到连接 Jetson 的那块网卡）：
# 2: enP8p1s0: <BROADCAST,MULTICAST,UP> ...
# 3: wlp2s0: <BROADCAST,MULTICAST,UP> ...

# 确认连通性
ping 172.20.10.3
```

### 编译与运行

```bash
./build.sh -m                    # 编译
sudo ./cmake_build/bin/rl_real_qmini   # 运行（Qmini 串口通信，无需网卡参数）
```

操作流程：A 站起 → Y 进入 Locomotion → 摇杆控制移动 → LB+X 退出 → B 趴下。

### 代码同步

本机修改代码后，同步到 Jetson 并远程编译：

```bash
ssh-copy-id qmini@172.20.10.3          # 首次：配置免密登录（需确认网卡已连通）
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

原理：`new_offset[i] = old_offset[i] + (current_q[i] − default_dof_pos[i])`

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

标定完成后同步回本机，并更新训练用 URDF：

```bash
# 将标定结果同步回开发机
scp qmini@172.20.10.3:/home/qmini/sim2real/rl_sar/policy/Qmini/base.yaml policy/Qmini/base.yaml

# 将 base.yaml 中的限位写入 URDF（用于重新训练）
python3 scripts/sync_limits_to_urdf.py            # 更新 URDF
python3 scripts/sync_limits_to_urdf.py --dry-run  # 只查看差异，不修改
python3 scripts/sync_limits_to_urdf.py --margin 2 # 限位向内缩 2°（安全裕度）
```

脚本会自动校验 min < max、检查左右腿对称性，并备份原 URDF。

### 关节镜像测试

验证实物与仿真的 URDF 关节方向、符号是否一致。Jetson 端以零力矩读取关节和 IMU，通过 UDP 发送到 PC；PC 端驱动 Gazebo 中固定悬空的机器人跟随实物运动。

```bash
# 先确认 PC 端 IP（Jetson 需要知道 PC 的地址）
ip link show                            # 找到连接 Jetson 的网卡名
ip addr show <网卡名>                   # 查看 PC 在该网卡上的 IP（通常为 172.20.10.2）

# PC 终端 1：启动 Gazebo（机器人固定悬空）
source install/setup.bash
ros2 launch rl_sar mirror.launch.py rname:=Qmini

# PC 终端 2：启动镜像接收
source install/setup.bash
ros2 run rl_sar rl_mirror_sim

# Jetson：启动镜像发送（填入上面查到的 PC IP）
sudo ./cmake_build/bin/rl_mirror_qmini <PC_IP>
# 例如：sudo ./cmake_build/bin/rl_mirror_qmini 172.20.10.2
```

PC 端打印实物与仿真的关节角度对比表，用于逐关节确认方向一致性。

### 运行日志分析

`rl_real_qmini` 运行时自动在 `policy/Qmini/` 下生成 CSV 日志，可用脚本可视化：

```bash
python3 scripts/plot_rl_log.py                           # 自动加载最新日志
python3 scripts/plot_rl_log.py policy/Qmini/rl_log_*.csv # 指定文件
```

## 配置说明

每个机器人的核心配置位于 `policy/<ROBOT>/base.yaml`，关键字段：

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
        policy.onnx
```

## 添加自定义机器人

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
    fsm_all.hpp        ← 注册新机器人

src/rl_sar/src/
    rl_real_<ROBOT>.cpp
```

参考 `go2w` 目录结构进行适配。

> **注意** — 使用本代码产生的所有风险及后果由使用者自行承担，操作前请确保已采取充分安全防护措施。
