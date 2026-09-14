# Dynamic Assembly：面向动态装配的机械臂目标跟踪与预测控制

基于 ROS2 的六自由度机械臂研究项目，围绕动态装配中的目标跟踪与运动控制，构建“目标状态估计与预测 → APF 局部参考规划 → 运动学 MPC”的仿真闭环，并提供强化学习（RL）残差控制扩展接口。

**当前进展：已完成仿真闭环验证，实机尚未验证。** 当前目标运动建模以固定高度平面为主，仿真跟踪效果不等同于完整的接触装配或实机装配验证。

## 项目功能与状态

| 模块 | 当前功能 | 状态 |
| --- | --- | --- |
| 目标输入 | 真实 ArUco 检测接入、模拟 ArUco 目标生成 | 已有接口与节点 |
| 状态估计 | 平面位置、速度估计，TF2 坐标变换及短时轨迹预测 | 已实现 |
| APF 规划 | 目标预测引导、静态球形障碍物斥力、前瞻点及参考路径生成 | 已实现 |
| 运动学 MPC | MoveIt 正运动学与雅可比、有限时域 QP、OSQP 求解、关节约束 | 已实现 |
| 控制输出 | JointTrajectory 发布、关节反馈、输入过期与求解异常回退 | 已实现 |
| RL 残差扩展 | 观测发布、外部动作接入、残差限幅及轨迹修正 | 接口已实现，默认不介入控制 |
| 实验分析 | 数据导出、MATLAB 绘图、MPC 求解耗时记录 |
| FPGA / 实机接口 | 通信占位、软件限位检查与看门狗逻辑 |调试中 |

## 系统架构

保留字符框图表示方式。实线表示基线闭环；下方 RL 框表示可选扩展，不代表默认启动或已经完成策略训练。

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                         ROS2 / Host Platform                            │
│                                                                         │
│  ┌─────────────────┐   ┌─────────────────┐   ┌──────────────────────┐   │
│  │ ArUco Input     │──▶│ State Estimator │──▶│ APF Local Planner    │   │
│  │ Camera / Fake   │   │ TF2 + KF        │   │ Lookahead / Path     │   │
│  └─────────────────┘   └─────────────────┘   └──────────┬───────────┘   │
│                                                         │               │
│                                                         ▼               │
│  ┌─────────────────┐   ┌─────────────────┐   ┌──────────────────────┐   │
│  │ Arm Simulation  │◀──│ Joint Trajectory│◀──│ Kinematic MPC        │   │
│  │ Gazebo / AR4    │   │ Controller      │   │ MoveIt + OSQP        │   │
│  └────────┬────────┘   └─────────────────┘   └──────────────────────┘   │
│           │                                                             │
│           └── /joint_states ──▶ APF / MPC / optional RL                 │
│                                                                         │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │ Optional RL Residual Extension (separate launch)                  │  │
│  │                                                                   │  │
│  │ Joint / Target / Path / MPC nominal ──▶ Observation (38 values)   │  │
│  │                                                 │                 │  │
│  │                                                 ▼                 │  │
│  │                                      External policy [planned]    │  │
│  │                                                 │                 │  │
│  │                                                 ▼                 │  │
│  │ MPC nominal ──▶ Residual fusion ◀── Action input (6 values)       │  │
│  │                        │                                          │  │
│  │                        └──▶ /rl/corrected_cmd                     │  │
│  │                             Controller output: OFF by default     │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

- 主入口 `assembly_system` 使用多线程执行器运行状态估计、APF 和 MPC 节点。
- APF 和 MPC 通过 MoveIt 加载机械臂模型；控制反馈来自 `/joint_states`。
- MPC 默认配置输出到 `/arm_controller/joint_trajectory`，同时发布 `/mpc_nominal_cmd`。
- RL 节点由独立启动文件加载；若未来将修正指令送入控制器，需要先调整输出路由，避免 MPC 和 RL 同时向控制器发送指令。
- `hardware_interface_node` 保留为后续实机扩展；当前 FPGA 读写为模拟占位，不属于已验证的仿真闭环。

## 核心方法

### 1. 目标状态估计与预测

接收 `ros2_aruco_interfaces/msg/ArucoMarkers`，按标记 ID 提取目标，并按需通过 TF2 转换至跟踪坐标系。


### 2. APF 局部参考规划

利用目标估计与预测、机械臂当前末端位置以及配置的静态球形障碍物，计算吸引与排斥作用，生成局部前瞻点及供 MPC 使用的参考路径。


### 3. 运动学 MPC

使用 MoveIt 提供的正运动学与雅可比，建立线性化运动学预测模型，以关节速度序列为优化变量，通过 OSQP 求解有限时域二次规划。

当前实现包括位置跟踪、姿态相关代价项、控制量与控制变化代价、终端代价，以及关节位置和速度约束。关节位置边界来自机器人模型；平面 yaw 跟踪可配置，当前 YAML 中关闭。

收到目标路径和关节状态后，MPC 滚动生成关节轨迹，并发布控制指令及名义轨迹。该实现属于运动学控制，不包含完整的关节动力学、力矩控制或接触力控制模型。

### 4. 强化学习残差控制

研究方向是在 MPC 名义控制的基础上学习小幅修正，用于后续研究模型误差或扰动补偿。**目前实现的是残差控制接口，不包含完整训练环境、奖励函数、训练脚本或已训练模型。**


## 目录结构

```text
dynamic_assembly/
├── README.md
├── CMakeLists.txt
├── package.xml
├── config/
│   └── params.yaml                    # 节点参数及部分启动默认值
├── include/dynamic_assembly/           # 节点声明与公共类型
├── src/
│   ├── main.cpp                       # 状态估计 / APF / MPC 多线程入口
│   ├── ekf_filter_node.cpp             # 平面状态估计、坐标变换与预测
│   ├── apf_planner_node.cpp            # APF 局部参考规划
│   ├── mpc_planner_node.cpp            # MoveIt 运动学与 OSQP MPC
│   ├── fake_aruco_publisher_node.cpp   # 模拟目标输入
│   ├── rl_residual_node.cpp            # RL 观测、动作与残差融合接口
│   └── hardware_interface_node.cpp     # 预留硬件接口与模拟读写
├── launch/
│   ├── bringup.launch.py               # AR4 仿真与主规划系统入口
│   ├── ekf_camera_aruco.launch.py      # 相机 / ArUco / 状态估计 / APF
│   ├── fake_aruco_tracking.launch.py   # 模拟目标 / 状态估计 / APF
│   └── rl_residual.launch.py           # 独立 RL 残差节点
├── scripts/
│   └── export_apf_bag_to_csv.py        # 实验数据导出
└── matlab/
    └── plot_apf_local_avoidance.m      # APF 实验绘图
```

## 主要话题

下表列出当前默认配置中的接口；使用不同参数或重映射时，以实际启动配置为准。

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/aruco_markers` | `ros2_aruco_interfaces/msg/ArucoMarkers` | 目标观测输入 |
| `/estimated_object_state` | `nav_msgs/msg/Odometry` | 目标状态估计，供 APF / RL 使用 |
| `/predicted_target_trajectory` | `nav_msgs/msg/Path` | 目标预测轨迹，供 APF 使用 |
| `/apf/lookahead_point` | `geometry_msgs/msg/PointStamped` | APF 前瞻点 |
| `/strategic_goal_path` | `nav_msgs/msg/Path` | APF 参考路径，供 MPC / RL 使用 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节反馈 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 控制器指令输入 |
| `/mpc_nominal_cmd` | `trajectory_msgs/msg/JointTrajectory` | MPC 名义轨迹 |
| `/rl/observation` | `std_msgs/msg/Float64MultiArray` | 38 维策略观测 |
| `/rl/external_action` | `std_msgs/msg/Float64MultiArray` | 外部策略的 6 维动作输入 |
| `/rl/residual_action` | `std_msgs/msg/Float64MultiArray` | 缩放限幅后的速度残差 |
| `/rl/corrected_cmd` | `trajectory_msgs/msg/JointTrajectory` | 修正轨迹，默认用于观察与调试 |

硬件预留模块另有 `/micro_trajectory_command`、`/robot_state` 和 `/emergency_stop` 接口，未接入当前默认控制链路。

## 参数与运行边界

参数集中在 `config/params.yaml`。下列数值是该文件当前配置，实际加载结果应通过运行时参数确认。

| 模块 | 参数 | 当前配置 | 含义 |
| --- | --- | --- | --- |
| 状态预测 | `prediction_steps / prediction_dt` | 50 / 0.02 s | 预测点数与步长 |
| APF | `period_ms` | 10 ms | 规划定时器周期 |
| MPC | `horizon / dt` | 40 / 0.005 s | 预测步数与模型步长；dt 同时用于定时器 |
| MPC | `timeout_ms` | 50 ms | 求解超时阈值 |
| MPC | `max_data_age` | 0.75 s | 目标路径与关节反馈过期阈值 |
| MPC | `max_joint_velocity` | 2.0 rad/s | 关节速度上限 |
| RL | `period_ms` | 50 ms | 残差节点定时器周期 |
| RL | `max_action_age` | 0.2 s | 外部动作过期阈值 |
| RL | `max_residual_velocity` | 0.05 rad/s | 残差速度限幅 |
| RL | `max_corrected_velocity` | 1.0 rad/s | 修正后速度限幅 |

- 定时器周期是配置值，不代表已达到对应的稳定运行频率。当前 MPC 的 5 ms 配置周期与 50 ms 超时阈值需要结合耗时统计评估。
- MPC 输入缺失或过期、求解失败或超时时，尝试发布位置保持、零速度的回退指令；无可用关节状态或历史轨迹时跳过回退指令。
- 软件约束、保持指令和预留看门狗不等同于经过实机验证的硬件急停。
- 当前节点代码使用仿真时间；复现时应确认 `/clock`、TF 和各输入时间基准一致。

## 环境与外部依赖

核心依赖包括 ROS2、rclcpp、TF2、Eigen、MoveIt 和 OSQP；ArUco 接口依赖 `ros2_aruco_interfaces`。相机输入使用 `usb_cam` 与 `ros2_aruco`，数据导出使用 ROS2 Python / rosbag2 相关包。

当前启动文件还依赖本仓库之外的 `ar4_moveit_config` 和 `ar4_gazebo_bringup`。仅下载本目录不足以构成完整仿真环境。

| 项目 | 已验证版本或来源 |
| --- | --- |
| 操作系统 / 是否使用 WSL | 本项目开发与测试均在WSL2环境中完成 |
| ROS2 发行版 | Jazzy |
| Gazebo / MoveIt 版本 | 同ROS2 |
| AR4 模型、MoveIt 配置、仿真包 | 正在完善中，后续将上传 |
真实相机模式使用以下 ROS2 软件包：

| 软件包 | 作用 |
|---|---|
| `usb_cam` | 读取 USB 摄像头并发布图像和相机信息 |
| `ros2_aruco` | 根据相机图像检测 ArUco 标记 |
| `ros2_aruco_interfaces` | 提供 `ArucoMarkers` 消息类型 |
| `tf2_ros` | 将相机坐标系下的目标位姿转换到 `world` 等跟踪坐标系 |
| `tf2_geometry_msgs` | 支持几何消息的 TF2 坐标变换 |
| OpenCV ArUco | ArUco 检测底层依赖，通常由 `ros2_aruco` 间接使用 |

本项目默认使用：

- ArUco 字典：`DICT_6X6_50`
- 目标标记 ID：`0`
- 标记尺寸：`0.10 m`
- 图像话题：`/image_raw`
- 相机信息话题：`/camera_info`
- 跟踪坐标系：`world`

启动真实相机前，需要准备相机标定文件，并在启动参数中设置：

camera_info_url: "file:///path/to/your/camera_calibration.yaml"

相机标定路径和相机坐标变换应按使用环境配置。当前配置中含本地标定文件路径，复现前需要替换。

## 操作说明

### 1. 环境安装与工作空间准备
本项目在 Linux / WSL 环境下开发，使用 ROS2、MoveIt、Gazebo、Eigen 和 OSQP。建议使用 Ubuntu 24.04 + ROS2 Jazzy；如果使用其他版本，请相应替换安装包名称和外部依赖版本。


### 2. 无相机仿真闭环

fake_aruco_tracking.launch.py 只启动模拟目标、状态估计和可选 APF，不启动 Gazebo 与 MPC；组合启动时需避免重复启动状态估计或 APF 节点。

终端 1：使用 ros2 launch dynamic_assembly bringup.launch.py 启动 moveit2
终端 2：不使用摄像头启动EKF相关节点 ros2 launch dynamic_assembly fake_aruco_tracking.launch.py 
终端 3：单独启动MPC节点 ：ros2 run dynamic_assembly mpc_planner_node --ros-args   --params-file /home/bran_24/ws_moveit2/src/dynamic_assembly/config/params.yaml
终端 4：可使用 ros2 topic list 查看 topic 运行情况

### 3. 停止流程与常见问题

暂无


## 后续工作

1. 整理依赖版本和可复现操作说明，补充仿真视频及定量结果。
2. 构建 RL 训练环境、奖励函数与训练流程，接入策略推理并完成基线对比。
3. 完善 RL 输入时效性、修正轨迹约束检查与控制器输出切换。
4. 扩展动态障碍物处理和整臂碰撞验证。
5. 完成真实机械臂通信、反馈与控制联调，并开展实机验证。
6. 按课题进展扩展接触装配、力控或更完整的目标姿态处理。

