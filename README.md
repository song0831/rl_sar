# rl_sar

机器人强化学习策略的仿真验证与实物部署框架，支持轮足与人形机器人。

---

## 支持列表

| 机器人 | rname | Policy | Gazebo | Real |
|--------|-------|--------|:------:|:----:|
| NIO ATOM01 | `ATOM01` | nio_lab | ✅ | ✅ |
| NIO Qmini | `Qmini` | nio_lab | ✅ | ✅ |
| Unitree G1 | `g1` | nio_lab / whole_body_tracking | ✅ | ✅ |
| Unitree Go2W | `go2w` | robot_lab | ✅ | ✅ |

---

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
# 编译全部
./build.sh

# 编译指定包
./build.sh package1 package2

# 仅硬件部署（不依赖 ROS）
./build.sh -m

# 清理构建产物
./build.sh -c
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

> **提示** — 首次启动 Gazebo 若无模型，执行：
> ```bash
> git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
> ```

### 4. 真实机器人

```bash
# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_<ROBOT> <NETWORK_INTERFACE>

# CMake（无 ROS 环境）
./cmake_build/bin/rl_real_<ROBOT> <NETWORK_INTERFACE>
```

### 5. 机载 Jetson 部署（以 Qmini 为例）

适用于将策略直接运行在机器人机载计算单元（NVIDIA Jetson）上的场景，无需 ROS 环境。

**5.1 安装系统依赖**

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
    libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev
```

**5.2 安装 LibTorch（Jetson 专用）**

项目提供脚本自动检测 JetPack 版本，安装对应 PyTorch wheel 并生成 LibTorch：

```bash
bash scripts/install_pytorch_jetson.sh
```

> **说明** — Jetson 平台上 ONNX Runtime 会被自动禁用（CMake 检测到 `/etc/nv_tegra_release`），仅使用 LibTorch 进行推理。

**5.3 编译（CMake 模式，无需 ROS）**

```bash
./build.sh -m
```

编译产物位于 `cmake_build/bin/`。

**5.4 确认网卡名**

```bash
ip link show
```

**5.5 运行**

```bash
./cmake_build/bin/rl_real_Qmini <NETWORK_INTERFACE>
# 例如：
./cmake_build/bin/rl_real_Qmini eth0
```

**5.6 操作流程**

启动后按以下顺序操作：

| 步骤 | 手柄 | 键盘 | 说明 |
|------|------|------|------|
| 1 | A | `0` | 缓慢站起（2 秒插值到站立姿态） |
| 2 | RB+↑ 或 Y | `1` | 切换到 Locomotion 运动模式 |
| 3 | LY/LX | `W/S` `A/D` | 前后 / 左右移动 |
| 4 | RX | `Q/E` | Yaw 旋转 |
| 5 | LB+X | `P` | 退出运动，切回 Passive 模式 |
| 6 | B | `9` | 缓慢趴下，返回初始姿态 |

> **注意** — 策略文件路径在编译时固定为项目根目录下的 `policy/`，Jetson 上运行时需保证 `policy/Qmini/` 目录存在且内容完整：
> ```
> policy/Qmini/
>     base.yaml
>     nio_lab/locomotion/
>         config.yaml
>         policy.onnx
> ```

---

## 控制方式

### 键盘 / 手柄

| 手柄 | 键盘 | 功能 |
|------|------|------|
| A | `0` | 运动到默认站立姿态 |
| B | `9` | 返回初始姿态 |
| X | `N` | 切换导航模式 |
| RB+Y | `R` | 重置 Gazebo 环境 |
| RB+X | `Enter` | Gazebo 运行 / 暂停 |
| LB+X | `P` | 电机 Passive 模式 |
| RB+↑ / Y | `1` | Locomotion |
| LY / LX | `W/S` `A/D` | 前后 / 左右移动 |
| RX | `Q/E` | Yaw 旋转 |
| — | `Space` | 速度清零 |

### 手柄诊断工具

不同品牌手柄的按键 id 可能不同，提供两个诊断脚本：

**真实机器人（直接读取 `/dev/input/js*`）**

```bash
# 默认读取 /dev/input/js0
python3 scripts/test_joystick.py

# 若手柄挂载在其他节点
python3 scripts/test_joystick.py /dev/input/js1
```

示例输出：

```
BUTTON  id= 4  按下  →  Gamepad::Y      [RL 运动模式]
BUTTON  id= 0  按下  →  Gamepad::A      [站起 GetUp]
AXIS    id= 7  DPadY    -0.999  方向键Y → R1+↑=RL模式备选
  ↳ Gamepad::RB_DPadUp [RL 运动模式（备选）]
```

若 id 与期望不符，修改 `src/rl_sar/src/rl_real_<ROBOT>.cpp` 中 `readJoystick()` 里对应的 `setBtn(<id>, ...)` 即可。

**Gazebo 仿真（通过 ROS2 `/joy` 话题）**

```bash
source install/setup.bash
python3 scripts/test_joy_ros.py
```

示例输出：

```
buttons[ 0]  按下  →  期望: A   → GetUp  ✅
buttons[ 1]  按下  →  期望: B   → GetDown  ✅
buttons[ 3]  按下  →  期望: X   → Passive  ✅
buttons[ 4]  按下  →  期望: Y   → RL模式  ✅
```

若 index 与期望不符，修改 `src/rl_sar/src/rl_sim.cpp` 中 `JoyCallback()` 里对应的 `buttons[N]` 即可。

---

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

> 参考 `go2w` 目录结构进行适配。

---

> [!CAUTION]
> 使用本代码产生的所有风险及后果由使用者自行承担，操作前请确保已采取充分安全防护措施。
