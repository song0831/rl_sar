# RL_SAR 项目代码解释文档

## 项目概述

**rl_sar** (Reinforcement Learning - Simulation And Real) 是一个用于机器人强化学习算法仿真验证和实体部署的框架。支持四足机器人、轮式机器人和人形机器人。

### 核心特性
- 支持多种仿真平台：IsaacGym、IsaacSim、Gazebo、MuJoCo
- 支持 ROS1 (Noetic) 和 ROS2 (Foxy/Humble)
- 支持推理引擎：libtorch 和 onnxruntime
- 支持平台：Linux 和 macOS
- 支持任务类型：运动控制（Locomotion）和舞蹈（Dance）

### 支持的机器人
- Unitree A1, Go2, Go2W, B2, B2W, G1
- FFTAI GR1T1, GR1T2
- zhinao L4W4
- Deeprobotics Lite3
- DDTRobot Tita

---

## 目录结构

```
rl_sar/
├── src/                          # 源代码目录
│   ├── rl_sar/                   # 主要项目源码
│   │   ├── library/              # 核心库
│   │   │   ├── core/             # 核心功能模块
│   │   │   └── thirdparty/       # 第三方库和机器人SDK
│   │   ├── fsm_robot/            # 各机器人的状态机定义
│   │   ├── include/              # 头文件
│   │   ├── src/                  # 主程序源文件
│   │   ├── scripts/              # Python工具脚本
│   │   └── test/                 # 单元测试
│   ├── robot_joint_controller/   # ROS关节控制器
│   └── robot_msgs/               # ROS消息定义
├── policy/                       # 预训练策略文件
├── build/                        # 编译输出目录
├── install/                      # ROS安装目录
├── library/                      # 外部库（推理运行时）
└── scripts/                      # 构建和下载脚本
```

---

## 核心库 (library/core)

### 1. rl_sdk (RL SDK 核心框架)

**文件位置**: `src/rl_sar/library/core/rl_sdk/`

#### 功能说明
RL SDK 是整个框架的核心，提供了机器人强化学习控制的基础架构。

#### 主要数据结构

##### RobotCommand<T>
机器人命令结构，用于向机器人发送控制指令：
```cpp
struct RobotCommand {
    struct MotorCommand {
        vector<int> mode;      // 电机模式
        vector<T> q;           // 目标关节位置
        vector<T> dq;          // 目标关节速度
        vector<T> tau;         // 前馈力矩
        vector<T> kp;          // 位置增益
        vector<T> kd;          // 阻尼增益
    } motor_command;
};
```

##### RobotState<T>
机器人状态结构，包含传感器数据：
```cpp
struct RobotState {
    struct IMU {
        vector<T> quaternion;      // 姿态四元数 [w,x,y,z]
        vector<T> gyroscope;       // 陀螺仪数据
        vector<T> accelerometer;   // 加速度计数据
    } imu;
    
    struct MotorState {
        vector<T> q;           // 当前关节位置
        vector<T> dq;          // 当前关节速度
        vector<T> ddq;         // 关节加速度
        vector<T> tau_est;     // 估计力矩
        vector<T> cur;         // 电机电流
    } motor_state;
};
```

#### 输入接口
支持键盘和手柄输入：
- 键盘映射：Num0-9, WASD, QE, Space, Enter 等
- 手柄映射：A/B/X/Y 按钮, LB/RB 触发器, 摇杆等

#### 核心类 RL
基类提供统一接口：
```cpp
class RL {
    virtual vector<float> Forward() = 0;           // 策略推理
    virtual void GetState(RobotState*) = 0;        // 获取状态
    virtual void SetCommand(const RobotCommand*) = 0; // 设置命令
};
```

---

### 2. FSM (有限状态机)

**文件位置**: `src/rl_sar/library/core/fsm/`

#### 功能说明
实现机器人行为的状态机框架，管理不同行为状态之间的切换。

#### 核心类

##### FSMState
状态基类，每个状态需实现：
```cpp
class FSMState {
    virtual void Enter() = 0;          // 进入状态时执行
    virtual void Run() = 0;            // 状态运行逻辑
    virtual void Exit() = 0;           // 退出状态时执行
    virtual string CheckChange() = 0;  // 检查是否需要切换状态
};
```

##### FSM
状态机管理器：
- `AddState()`: 添加状态
- `SetInitialState()`: 设置初始状态
- `RequestStateChange()`: 请求状态切换
- `Run()`: 执行状态机主循环

#### 状态切换流程
1. `NORMAL` 模式：执行当前状态的 `Run()`
2. `CheckChange()` 返回新状态名时触发切换
3. `CHANGE` 模式：执行 `Exit()` → `Enter()` → `Run()`

---

### 3. inference_runtime (推理运行时)

**文件位置**: `src/rl_sar/library/core/inference_runtime/`

#### 功能说明
提供神经网络模型推理接口，支持 PyTorch (TorchScript) 和 ONNX 两种后端。

#### 核心类

##### Model (抽象接口)
```cpp
class Model {
    virtual bool load(const string& model_path) = 0;  // 加载模型
    virtual bool is_loaded() const = 0;                // 检查是否已加载
    virtual vector<float> forward(
        const vector<vector<float>>& inputs) = 0;      // 前向推理
    virtual string get_model_type() const = 0;         // 获取模型类型
};
```

##### TorchModel (PyTorch实现)
- 加载 `.pt` 或 `.jit` TorchScript 模型
- 使用 libtorch 进行推理
- 支持 CPU 和 CUDA

##### ONNXModel (ONNX实现)
- 加载 `.onnx` 模型文件
- 使用 onnxruntime 进行推理
- 自动选择可用的执行提供程序（CUDA/CPU）

#### 使用示例
```cpp
auto model = make_shared<TorchModel>();
model->load("policy.pt");
vector<float> output = model->forward({{obs1, obs2, ...}});
```

---

### 4. observation_buffer (观测缓冲区)

**文件位置**: `src/rl_sar/library/core/observation_buffer/`

#### 功能说明
管理历史观测数据的循环缓冲区，用于需要时序信息的策略。

#### 核心类 ObservationBuffer

##### 初始化参数
```cpp
ObservationBuffer(
    int num_envs,                    // 环境数量
    const vector<int>& obs_dims,     // 各观测组件维度
    int history_length,               // 历史长度
    const string& priority           // 优先级模式："time"或"term"
);
```

##### 主要方法
- `reset()`: 重置指定环境的缓冲区
- `insert()`: 插入新观测数据
- `get_obs_vec()`: 获取指定索引的观测向量

#### 优先级模式
- **time**: 按时间顺序排列 [t-n, t-n+1, ..., t]
- **term**: 按观测项分组 [obs1_history, obs2_history, ...]

#### 数据结构
```
obs_buf[env_id][time_step][obs_component]
```

---

### 5. motion_loader (运动数据加载器)

**文件位置**: `src/rl_sar/library/core/motion_loader/`

#### 功能说明
用于舞蹈和模仿任务，加载和播放运动捕捉数据（CSV格式）。

#### 核心类 MotionLoader

##### CSV 数据格式
每行表示一帧：
```
root_pos_x, root_pos_y, root_pos_z, 
root_quat_x, root_quat_y, root_quat_z, root_quat_w,
joint_0, joint_1, ..., joint_N
```

##### 主要方法
- `Update(time)`: 更新到指定时间
- `Reset()`: 重置运动并对齐 yaw 角
- `GetJointPos()`: 获取插值后的关节位置
- `GetJointVel()`: 获取插值后的关节速度
- `GetRootQuat()`: 获取根节点四元数
- `GetAnchorQuat()`: 获取躯干四元数（用于G1）

#### 插值方法
使用线性插值在相邻帧之间生成平滑的运动轨迹。

---

### 6. vector_math (向量数学库)

**文件位置**: `src/rl_sar/library/core/vector_math/`

#### 功能说明
提供四元数、欧拉角和向量的数学运算。

#### 主要功能
- 四元数运算：乘法、共轭、旋转向量
- 欧拉角转换：四元数 ↔ 欧拉角（ZYX顺序）
- 向量运算：归一化、点积、叉积
- 实用函数：wrap_to_pi（角度归一化）

#### 关键函数
```cpp
vector<float> quat_multiply(quat1, quat2);  // 四元数乘法
vector<float> quat_rotate(quat, vec);       // 用四元数旋转向量
vector<float> quat_to_euler(quat);          // 四元数转欧拉角
vector<float> euler_to_quat(roll,pitch,yaw);// 欧拉角转四元数
```

---

### 7. loop (循环管理器)

**文件位置**: `src/rl_sar/library/core/loop/`

#### 功能说明
管理多个定时循环任务，确保精确的执行频率。

#### 核心类 LoopFunc

##### 创建循环
```cpp
auto loop = make_shared<LoopFunc>(
    "loop_name",
    dt,                    // 时间步长（秒）
    callback_function      // 回调函数
);
loop->Start();
```

#### 特性
- 每个循环在独立线程中运行
- 自动补偿执行时间以维持恒定频率
- 支持启动/停止控制

---

### 8. logger (日志工具)

**文件位置**: `src/rl_sar/library/core/logger/`

#### 功能说明
提供彩色终端输出的日志工具。

#### 日志级别
```cpp
LOGGER::INFO     // 蓝色 - 一般信息
LOGGER::NOTE     // 绿色 - 重要提示
LOGGER::WARNING  // 黄色 - 警告
LOGGER::ERROR    // 红色 - 错误
```

#### 使用示例
```cpp
cout << LOGGER::INFO << "Robot initialized" << endl;
cout << LOGGER::ERROR << "Connection failed" << endl;
```

---

## FSM 状态机实现 (fsm_robot)

每个机器人都有专门的状态机实现，定义了特定的行为状态。

### 通用状态类型

#### 1. RLFSMStatePassive (被动模式)
- **功能**: 电机零力矩状态，仅提供阻尼
- **用途**: 安全模式，手动移动机器人
- **切换**: 按 Num0/A 切换到起立状态

#### 2. RLFSMStateGetUp (起立状态)
- **功能**: 从倒地/被动模式平滑过渡到站立姿态
- **逻辑**: 使用插值实现平滑运动
  - 阶段1: 预起立姿态（1秒）
  - 阶段2: 到达默认站立姿态（2秒）
- **切换**: 完成后自动进入 RL 控制或站立状态

#### 3. RLFSMStateRL (强化学习控制)
- **功能**: 执行强化学习策略
- **流程**:
  1. 收集观测数据（IMU、关节状态、命令）
  2. 策略推理得到动作
  3. 缩放和限幅动作
  4. 计算 PD 控制器输出
- **切换**: 按 Num9/B 切换到下蹲状态

#### 4. RLFSMStateGetDown (下蹲状态)
- **功能**: 平滑过渡到安全下蹲姿态
- **逻辑**: 插值到卧姿（1秒）
- **切换**: 完成后进入被动模式

### 特定机器人状态

#### G1 人形机器人额外状态

##### RLFSMStateDance (舞蹈状态)
- **功能**: 播放预录制的舞蹈动作
- **使用**: MotionLoader 加载 CSV 文件
- **控制**: 
  - 通过 nav 模式调整速度
  - 支持循环播放

##### RLFSMStateMimic (模仿状态)
- **功能**: 跟踪参考运动轨迹
- **特点**: 使用 anchor（躯干）作为跟踪目标而非 root

---

## 主程序源文件 (src)

### 1. rl_sim.cpp (Gazebo仿真)

**文件位置**: `src/rl_sar/src/rl_sim.cpp`

#### 功能说明
在 Gazebo 仿真器中运行强化学习策略，支持 ROS1/ROS2。

#### 核心类 RL_Sim

##### 初始化流程
1. 解析命令行参数（机器人类型、策略名称）
2. 加载配置文件（YAML）
3. 初始化 ROS 节点和话题
4. 启动关节控制器
5. 加载神经网络模型
6. 创建并初始化 FSM

##### ROS 接口

**订阅话题**:
- `/gazebo/imu`: IMU 数据
- `/joint_states`: 关节状态
- `/cmd_vel`: 速度命令（键盘控制）
- `/joy`: 手柄输入

**发布话题**:
- `/robot_command`: 关节命令
- 各关节控制器命令话题

**服务客户端**:
- `/gazebo/pause_physics`: 暂停仿真
- `/gazebo/unpause_physics`: 继续仿真
- `/gazebo/reset_world`: 重置世界

##### 执行循环
- **loop_keyboard** (100Hz): 处理键盘输入
- **loop_rl** (50Hz): 执行策略推理
- **loop_control** (200Hz): 发送控制命令
- **loop_plot** (可选): 可视化数据

---

### 2. rl_sim_mujoco.cpp (MuJoCo仿真)

**文件位置**: `src/rl_sar/src/rl_sim_mujoco.cpp`

#### 功能说明
使用 MuJoCo 物理引擎进行仿真，不依赖 ROS。

#### 核心类 RL_Sim_Mujoco

##### 特点
- 直接使用 MuJoCo C API
- 内置可视化窗口（GLFW）
- 支持键盘和手柄输入
- 无需 ROS 环境

##### MuJoCo 集成
```cpp
mjModel* model;    // 模型定义
mjData* data;      // 仿真数据
mjvCamera camera;  // 相机视角
mjvOption option;  // 渲染选项
```

##### 控制流程
1. 从 MuJoCo 读取传感器数据
2. 执行 FSM 和策略推理
3. 设置 MuJoCo 执行器控制信号
4. 单步仿真
5. 渲染可视化

---

### 3. rl_real_*.cpp (实体机器人)

**文件位置**: `src/rl_sar/src/rl_real_*.cpp`

每个机器人有专门的实体控制程序：
- `rl_real_a1.cpp`: Unitree A1
- `rl_real_go2.cpp`: Unitree Go2
- `rl_real_g1.cpp`: Unitree G1
- `rl_real_l4w4.cpp`: zhinao L4W4
- `rl_real_lite3.cpp`: Deeprobotics Lite3

#### 通用结构

##### 初始化
1. 加载配置文件
2. 连接机器人（UDP/SDK）
3. 加载神经网络模型
4. 创建 FSM 状态机

##### 通信接口
- **UDP**: A1（unitree_legged_sdk）
- **DDS**: Go2, G1（unitree_sdk2）
- **自定义协议**: L4W4, Lite3

##### 主循环
```cpp
while(running) {
    GetState(&robot_state);      // 1. 读取传感器
    fsm.Run();                   // 2. 执行状态机
    SetCommand(&robot_command);  // 3. 发送命令
    usleep(2000);                // 4. 等待（500Hz）
}
```

##### 安全机制
- 关节位置/速度限制
- 力矩限制
- 紧急停止（Ctrl+C）
- 状态检查和错误处理

---

## 机器人关节控制器 (robot_joint_controller)

**文件位置**: `src/robot_joint_controller/`

### 功能说明
Gazebo 和 ROS 之间的接口，实现关节级 PD 控制器。

### 核心类 RobotJointController

#### 控制算法
```cpp
tau = kp * (q_des - q_act) + kd * (dq_des - dq_act) + tau_ff
```

其中：
- `q_des`, `dq_des`: 期望位置和速度
- `q_act`, `dq_act`: 实际位置和速度
- `tau_ff`: 前馈力矩
- `kp`, `kd`: PD 增益（可动态调整）

#### ROS 接口

**订阅话题**: `~command` (robot_msgs/MotorCommand)
```
q: 目标位置
dq: 目标速度
kp: 位置增益
kd: 阻尼增益
tau: 前馈力矩
```

**发布话题**: `~state` (robot_msgs/MotorState)
```
q: 当前位置
dq: 当前速度
tau_est: 估计力矩
```

#### 实时性
- 使用 `realtime_tools` 确保实时性能
- 非实时回调写入，实时线程读取
- 状态发布使用实时发布器

---

## 策略配置文件 (policy)

### 目录结构
```
policy/
├── {robot_name}/
│   ├── base.yaml                    # 机器人基础配置
│   └── {policy_name}/
│       ├── config.yaml              # 策略配置
│       ├── model.pt / model.onnx    # 神经网络模型
│       └── motion.csv (可选)        # 运动数据
```

### 配置文件详解

#### base.yaml
机器人硬件参数：
```yaml
robot_name:
  num_of_dofs: 12                    # 自由度数量
  joint_names: [...]                 # 关节名称列表
  default_dof_pos: [...]             # 默认关节位置
  torque_limits: [...]               # 力矩限制
```

#### config.yaml
策略参数：
```yaml
policy_name:
  model_name: "model.pt"             # 模型文件名
  
  # 观测配置
  num_observations: 45               # 观测维度
  observations: ["ang_vel", "gravity_vec", ...]  # 观测组件
  observations_history: [5,4,3,2,1,0]  # 历史索引
  observations_history_priority: "time"  # 优先级模式
  
  # 动作配置
  action_scale: [0.125, ...]         # 动作缩放因子
  clip_actions_lower/upper: [...]    # 动作限幅
  
  # 控制增益
  rl_kp: [20.0, ...]                 # RL策略使用的kp
  rl_kd: [0.5, ...]                  # RL策略使用的kd
  fixed_kp: [80.0, ...]              # 固定控制使用的kp
  fixed_kd: [3.0, ...]               # 固定控制使用的kd
  
  # 命令缩放
  lin_vel_scale: 2.0                 # 线速度缩放
  ang_vel_scale: 0.25                # 角速度缩放
  commands_scale: [2.0, 2.0, 0.25]   # 命令缩放
```

### 观测组件说明
- `ang_vel`: 机体角速度 (3维)
- `gravity_vec`: 重力向量 (3维)
- `commands`: 速度命令 (3维: vx, vy, vyaw)
- `dof_pos`: 关节位置 (num_dofs维)
- `dof_vel`: 关节速度 (num_dofs维)
- `actions`: 上一步动作 (num_dofs维)

---

## 工具脚本 (scripts)

### 1. convert_policy.py

**文件位置**: `src/rl_sar/scripts/convert_policy.py`

#### 功能
PyTorch (.pt) 和 ONNX (.onnx) 模型格式相互转换。

#### 使用方法
```bash
# 自动检测格式并转换
python convert_policy.py input_model.pt

# 指定输出文件名
python convert_policy.py input_model.pt -o output_model.onnx
```

#### 转换流程
1. 检测输入格式
2. 自动推断输入输出维度
3. 创建虚拟输入进行转换
4. 验证转换结果
5. 保存输出模型

#### 支持的转换
- PT → ONNX: 使用 `torch.onnx.export()`
- ONNX → PT: 使用 ONNX 后端和 TorchScript

---

### 2. actuator_net.py

**文件位置**: `src/rl_sar/scripts/actuator_net.py`

#### 功能
执行器网络（Actuator Network）训练和转换工具。

#### 用途
模拟真实电机的延迟和动力学特性，提高 sim-to-real 迁移效果。

---

### 3. build.sh

**文件位置**: `build.sh`

#### 功能
项目构建脚本，统一管理编译流程。

#### 使用选项
```bash
./build.sh                # 编译所有ROS包
./build.sh pkg1 pkg2      # 编译指定包
./build.sh -c             # 清理构建文件
./build.sh -m             # 使用CMake编译（无ROS）
./build.sh -mj            # 编译MuJoCo支持
```

#### 构建流程
1. 检查和下载推理运行时库
2. （可选）下载 MuJoCo 库
3. 下载机器人描述文件
4. 编译 ROS 包或 CMake 项目
5. 创建符号链接

---

### 4. download_*.sh

**文件位置**: `scripts/download_*.sh`

#### download_inference_runtime.sh
自动下载和配置推理运行时库：
- libtorch (CPU/CUDA)
- onnxruntime (CPU/CUDA)

根据系统架构和 CUDA 版本选择合适的版本。

#### download_mujoco.sh
下载 MuJoCo 物理引擎库（3.2.7版本）。

#### download_robot_descriptions.sh
下载各机器人的 URDF/Xacro 描述文件。

---

## 测试文件 (test)

### 1. test_inference_runtime.cpp
测试神经网络推理功能：
- 模型加载
- 前向传播
- 性能测试

### 2. test_observation_buffer.cpp
测试观测缓冲区：
- 缓冲区初始化
- 数据插入和重置
- 历史数据提取

### 3. test_vector_math.cpp
测试数学库：
- 四元数运算
- 欧拉角转换
- 向量操作

---

## 编译系统

### CMakeLists.txt

**主配置**: `src/rl_sar/CMakeLists.txt`

#### 编译选项
```cmake
option(USE_ROS1 "Build with ROS1" OFF)
option(USE_ROS2 "Build with ROS2" OFF)
option(USE_TORCH "Build with LibTorch" ON)
option(USE_ONNX "Build with ONNX Runtime" ON)
option(USE_MUJOCO "Build with MuJoCo" OFF)
```

#### 目标程序
- **rl_sim**: Gazebo 仿真程序（需要 ROS）
- **rl_sim_mujoco**: MuJoCo 仿真程序
- **rl_real_{robot}**: 各机器人实体控制程序

#### 依赖库
- **必需**: yaml-cpp, Eigen3, spdlog, fmt, TBB
- **ROS1**: roscpp, sensor_msgs, geometry_msgs
- **ROS2**: rclcpp, sensor_msgs, geometry_msgs
- **推理**: libtorch / onnxruntime
- **仿真**: MuJoCo

---

## 消息定义 (robot_msgs)

### MotorCommand
```
float32[] q        # 目标位置
float32[] dq       # 目标速度
float32[] kp       # 位置增益
float32[] kd       # 阻尼增益
float32[] tau      # 前馈力矩
```

### MotorState
```
float32[] q        # 当前位置
float32[] dq       # 当前速度
float32[] ddq      # 加速度
float32[] tau_est  # 估计力矩
float32[] cur      # 电机电流
```

### RobotCommand
```
MotorCommand motor_command
```

### RobotState
```
MotorState motor_state
geometry_msgs/Quaternion quaternion
geometry_msgs/Vector3 gyroscope
geometry_msgs/Vector3 accelerometer
```

---

## 运行流程

### 仿真运行流程

#### 1. 启动 Gazebo
```bash
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=go2
```

#### 2. 运行控制程序
```bash
ros2 run rl_sar rl_sim go2 robot_lab
```

#### 3. 交互控制
- 键盘: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- 手柄: 连接支持的游戏手柄

### 实体运行流程

#### 1. 编译实体程序
```bash
./build.sh -m
```

#### 2. 上传到机器人
```bash
scp -r cmake_build robot@robot_ip:~/
```

#### 3. 运行
```bash
ssh robot@robot_ip
cd cmake_build/bin
./rl_real_go2 go2 robot_lab
```

---

## 安全注意事项

### 仿真安全
- 按 Enter 暂停/继续仿真
- 按 R 重置世界
- 按 Escape 退出程序

### 实体运行安全
⚠️ **警告**：实体机器人运行前必须采取安全措施！

1. **环境准备**
   - 确保周围无障碍物和人员
   - 使用防护围栏
   - 准备紧急停止装置

2. **启动检查**
   - 检查机器人状态
   - 验证通信连接
   - 确认电池电量

3. **运行监控**
   - 随时准备按 P 键进入被动模式
   - 随时准备 Ctrl+C 紧急停止
   - 监控异常振动和声音

4. **紧急响应**
   - P键: 立即切换到被动模式（零力矩）
   - Ctrl+C: 终止程序
   - 物理急停按钮: 切断电源

---

## 扩展开发

### 添加新机器人

#### 1. 创建配置文件
```yaml
# policy/{robot_name}/base.yaml
{robot_name}:
  num_of_dofs: XX
  joint_names: [...]
  default_dof_pos: [...]
  # ... 其他参数
```

#### 2. 创建 FSM 定义
```cpp
// fsm_robot/fsm_{robot_name}.hpp
namespace {robot_name}_fsm {
    class RLFSMStatePassive : public RLFSMState { ... };
    class RLFSMStateGetUp : public RLFSMState { ... };
    // ... 其他状态
}
```

#### 3. 实现实体控制
```cpp
// src/rl_real_{robot_name}.cpp
class RL_Real_{RobotName} : public RL {
    void GetState(RobotState<float>* state) override;
    void SetCommand(const RobotCommand<float>* cmd) override;
    vector<float> Forward() override;
};
```

#### 4. 添加到 fsm_all.hpp
```cpp
#include "fsm_{robot_name}.hpp"
// 在 CreateRobotFSM 中添加分支
```

### 添加新策略

#### 1. 训练策略
使用 IsaacGym/IsaacSim 训练神经网络策略。

#### 2. 导出模型
```python
# 导出为 TorchScript
traced = torch.jit.trace(model, example_input)
traced.save("model.pt")

# 或使用转换工具导出为 ONNX
python convert_policy.py model.pt
```

#### 3. 创建配置
```yaml
# policy/{robot}/{policy_name}/config.yaml
{robot}/{policy_name}:
  model_name: "model.pt"
  num_observations: XX
  # ... 观测和动作配置
```

#### 4. 测试
```bash
# 先在仿真中测试
ros2 run rl_sar rl_sim {robot} {policy_name}

# 确认无误后在实体上测试
./rl_real_{robot} {robot} {policy_name}
```

---

## 性能优化

### 推理性能
- 使用 ONNX 模型（通常比 TorchScript 快）
- 启用 CUDA 加速（如果有 GPU）
- 减小批处理大小
- 使用量化模型（可选）

### 控制频率
- 仿真: 50Hz (策略) + 200Hz (控制)
- 实体: 200-500Hz (根据机器人性能)

### 内存优化
- 使用 `reserve()` 预分配向量
- 避免频繁内存分配
- 使用循环缓冲区

---

## 故障排查

### 编译错误
1. **找不到库**: 运行 `./build.sh` 自动下载
2. **ROS 版本**: 检查 ROS_DISTRO 环境变量
3. **CUDA 版本**: 确保 libtorch/onnxruntime 与 CUDA 版本匹配

### 运行时错误
1. **模型加载失败**: 检查文件路径和格式
2. **关节名称不匹配**: 对比 URDF 和配置文件
3. **通信超时**: 检查网络连接和 IP 地址

### 实体控制问题
1. **机器人不动**: 检查是否在被动模式，按 Num0/A 起立
2. **异常振动**: 降低 PD 增益，检查动作缩放
3. **失去平衡**: 调整策略参数，重新训练

---

## 相关资源

### 文档
- [README.md](README.md): 项目说明
- [README_CN.md](README_CN.md): 中文文档

### 训练框架
- [robot_lab](https://github.com/fan-ziqi/robot_lab): IsaacLab 训练环境

### 社区
- [GitHub Discussions](https://github.com/fan-ziqi/rl_sar/discussions)
- [Discord](http://www.robotsfan.com/dc_rl_sar)

---

## 许可证

本项目采用 Apache 2.0 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 贡献者

详见 [CONTRIBUTORS.md](CONTRIBUTORS.md)

---

**文档版本**: 对应项目版本 v3.x
**最后更新**: 2026-01-05
