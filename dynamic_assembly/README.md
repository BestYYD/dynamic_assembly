# Dynamic Assembly：面向动态装配的机械臂目标跟踪与预测控制

基于 ROS2 的六自由度机械臂研究项目，围绕动态装配中的目标跟踪与运动控制，构建“目标状态估计与预测 → APF 局部参考规划 → 运动学 MPC”的仿真闭环，并提供强化学习（RL）残差控制扩展接口。

**当前状态：实验性目标跟踪代码，RL 仅有接口，实机与接触装配尚未验证。** 本次于 2026-10-22 同步本地主包源码并核对文档，没有修改控制算法、启动机械臂或执行完整仿真回归。当前目标运动建模以固定高度平面为主；已有仿真开发与调试记录不等于完整的接触装配或实机安全验证。运行前请先阅读本文的“已知问题与运行限制”。

本仓库只同步 `dynamic_assembly` 主包。AR4 模型、Gazebo / MoveIt 配置、ArUco 源码及 `robot_trajectory_tools` 是外部依赖，按本次同步范围不上传。**仅克隆此仓库不能得到完整 AR4 仿真工作空间。** 备份入口见仓库根目录的 [BACKUPS.md](../BACKUPS.md)。

## 项目功能与状态

| 模块 | 当前功能 | 状态 |
| --- | --- | --- |
| 目标输入 | 真实 ArUco 检测接入、模拟 ArUco 目标生成 | 已有接口与节点 |
| 状态估计 | 平面位置、速度估计，TF2 坐标变换及短时轨迹预测 | 已实现 |
| APF 规划 | 目标预测引导、静态障碍物的 XY 平面斥力、前瞻点及参考路径生成 | 已实现；非整臂碰撞保证 |
| 运动学 MPC | MoveIt 正运动学与雅可比、有限时域 QP、OSQP 求解、关节约束 | 已实现 |
| 控制输出 | JointTrajectory 发布、关节反馈、输入过期与求解异常回退 | 已实现 |
| RL 残差扩展 | 观测发布、外部动作接入、残差限幅及轨迹修正 | 接口已实现，默认不介入控制 |
| 实验分析 | 数据导出、MATLAB 绘图、MPC 求解耗时记录 | 工具已包含；定量结果未随本次发布 |
| FPGA / 实机接口 | 通信占位、软件限位检查与看门狗逻辑 | 模拟占位，未接入默认控制链 |

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

文件名沿用 EKF，但当前状态是 `[x, y, vx, vy]`，采用线性匀速状态转移和线性位置观测，实际是线性卡尔曼滤波。Z 固定为 `target_plane_z`。输出朝向主要由运动方向构造，不是完整的工件姿态估计，不能直接当作装配孔轴姿态。

### 2. APF 局部参考规划

利用目标估计与预测、机械臂当前末端位置以及配置的静态障碍物，计算吸引与排斥作用，生成局部前瞻点及供 MPC 使用的参考路径。当前本地实现计算斥力时将 Z 方向差置零，属于 XY 平面的圆形障碍物模型（空间上相当于柱状影响区域），不是三维球体距离。路径步长保留合力幅值并限制最大长度。

默认障碍物中心为 `(0.32, 0.05, 0.04)` m、半径为 `0.04` m。APF 只处理末端参考，不进行整臂、自碰撞或真实接触验证。RViz 标记也不等于 Gazebo 碰撞体或 MoveIt PlanningScene 对象。


### 3. 运动学 MPC

使用 MoveIt 提供的正运动学与雅可比，建立线性化运动学预测模型，以关节速度序列为优化变量，通过 OSQP 求解有限时域二次规划。

当前实现包括位置跟踪、姿态相关代价项、控制量与控制变化代价、终端代价，以及关节位置和速度约束。关节位置边界来自机器人模型；平面 yaw 跟踪可配置，当前 YAML 中关闭。

开启当前 yaw 项主要用于保持初始化时的工具 yaw，不是跟踪工件真实 yaw。加速度目前没有完整硬约束；直接发布的 MPC 轨迹不会因为使用了 MoveIt FK / 雅可比，就自动经过 MoveIt 碰撞检查。

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
- 多个节点会强制开启 `use_sim_time`，同时仍混用 wall timer、`steady_clock` 和 ROS 时间；不能将本版本直接用于暂停 / 加速 / 重置式强化学习训练。
- 参数通常在构造时读入；修改 ROS 参数服务器不代表成员变量和定时器已经更新。未实现动态更新回调的参数按重启生效处理。

## 环境与外部依赖

核心依赖包括 ROS2、rclcpp、TF2、Eigen、MoveIt 和 OSQP；ArUco 接口依赖 `ros2_aruco_interfaces`。相机输入使用 `usb_cam` 与 `ros2_aruco`，数据导出使用 ROS2 Python / rosbag2 相关包。

当前启动文件还依赖本仓库之外的 `ar4_moveit_config` 和 `ar4_gazebo_bringup`，以及它们引用的描述、IKFast、公共启动和控制器包。开发工作区中的 Gazebo 启动还引用 `robot_trajectory_tools`。这些文件不属于本次主包同步范围。

以下是同步时在开发机上读取的已安装软件包版本，不是一个已经完成干净环境回归的依赖锁文件。工作区 source overlay 可能覆盖系统版本。

| 项目 | 本机已安装环境 / 来源 |
| --- | --- |
| 操作系统 | Windows 上的 WSL2，Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic / gz-sim 8.14.0；ros_gz 1.0.22 |
| 控制插件 | gz_ros2_control 1.2.19；本机保留旧 ign 插件名的兼容别名 |
| MoveIt | 已安装 moveit_core 2.12.4 |
| OSQP | 系统头文件报告 C API 0.6.2；osqp_vendor 软件包版本 0.2.0，两者不是同一版本号 |
| usb_cam | 0.8.1 |
| AR4 及轨迹工具 | 外部工作区源码；上游提交与本地修改尚未形成可复现锁文件，不随本仓库上传 |
| ArUco | 外部 `ros2_aruco` 和 `ros2_aruco_interfaces`；源码来源提交尚未锁定 |

建议继续使用 Jazzy + Gazebo Harmonic。当前低维状态残差研究不需要 Isaac Sim；本仓库也不提供 Isaac Lab 训练任务。参考 [Gazebo / ROS 兼容说明](https://gazebosim.org/docs/harmonic/ros_installation/) 与 [gz_ros2_control 文档](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)。不要直接升级 OSQP 主版本而不检查现有 `OSQPData` / `OSQPWorkspace` C API。

相机标定路径和相机坐标变换应按使用环境配置。当前配置中含本地标定文件路径，复现前需要替换。

## 操作说明

以下命令面向 **WSL / Ubuntu 的 Bash 终端及纯仿真**，不面向真实机械臂。各入口和参数已经按源码核对，但本次文档同步没有执行完整仿真。它们不是已通过干净机器端到端验收的一键安装脚本。

### 1. 获取与构建主包

新工作区可以按下面的目录布局获取主包；已有工作区不要再放入第二份同名 ROS 包。

```bash
mkdir -p ~/ws_dynamic_assembly/src
cd ~/ws_dynamic_assembly
git clone https://github.com/BestYYD/dynamic_assembly.git src/dynamic_assembly_repo
source /opt/ros/jazzy/setup.bash
```

先准备并 source 外部依赖所在的已构建工作区。至少需要 Eigen、MoveIt、兼容的 OSQP、`ros2_aruco_interfaces`；运行现有启动文件还需要 AR4 相关包、Gazebo、ros2_control、`moveit_configs_utils`、`ament_index_python`、`launch_ros` 和 PyYAML。相机模式另外需要 `usb_cam`、`ros2_aruco`、相机标定与外参；启动中的曝光设置还可能使用 `v4l2-ctl`。

`package.xml` 尚未完整声明所有启动时依赖，`rosdep` 成功不代表完整仿真依赖已齐备。外部源码版本尚未锁定，不能在这里给出未经确认的依赖仓库提交号。

外部依赖已经可用后，构建主包：

```bash
colcon build --packages-select dynamic_assembly --symlink-install
source install/setup.bash
ros2 pkg executables dynamic_assembly
```

每个新终端都需要加载 ROS 和实际工作区的 `install/setup.bash`。例如当前开发工作区为 `/home/bran_24/ws_moveit2`，但其他机器不应照搬这个绝对路径。

### 2. 启动文件的职责

| 入口 | 实际启动内容 | 不包含 / 注意事项 |
| --- | --- | --- |
| `bringup.launch.py` | 外部 AR4 Gazebo 栈、可选真实相机 / ArUco、`assembly_system` 内的 KF / APF / MPC | 没有模拟目标节点；有下文所述参数重复声明问题 |
| `fake_aruco_tracking.launch.py` | 模拟目标、KF、默认开启的 APF | 不启动 Gazebo、MPC 或控制器 |
| `ekf_camera_aruco.launch.py` | 相机 / ArUco / 相机静态 TF / KF / 可选 APF | 不启动 Gazebo、MPC 或控制器；需核对标定与外参 |
| `rl_residual.launch.py` | 独立残差节点 | 不启动策略训练、MPC 或仿真；默认零残差且不接管控制器 |

**不要先运行 `bringup.launch.py`，再重复运行完整的 KF / APF / MPC。** 这会产生重复节点和重复指令发布者。模拟 ArUco 与真实 ArUco 也不应同时发布到同一目标话题。

### 3. 无相机分模块仿真调试

以下分模块方式不启动 `assembly_system`，用于避开主入口参数声明冲突并分别检查各环节。前提是外部 AR4 包及其依赖已配置好；本次未运行验证这一组合。

终端 1：仅启动外部仿真栈，并检查时间、控制器和关节反馈。

```bash
ros2 launch ar4_gazebo_bringup main.launch.py
# 在另一个已加载环境的终端检查：
ros2 topic echo /clock --once
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

应能看到有效的 `/clock`、六关节反馈，以及 active 的 `joint_state_broadcaster`、`arm_controller`。否则不要继续启用 MPC。

终端 2：启动一个模拟目标输入源、一个 KF 和一个 APF。

```bash
ros2 launch dynamic_assembly fake_aruco_tracking.launch.py \
  use_fake_aruco:=true use_aruco:=false use_apf:=true
```

默认目标中心约为 `(0.43, 0.05, 0.12)` m，XY 幅度均为 `0.035` m，角频率均为 `0.4` rad/s；由 launch 参数和 `params.yaml` 共同设置。检查 `/estimated_object_state`、`/predicted_target_trajectory`、`/strategic_goal_path` 及其 frame。

终端 3：MPC 需要参数 YAML 和机器人描述。不要把“只传一个 params.yaml”当成在所有环境下都足够。下面通过一个临时 Python 启动描述传入 MoveIt 模型参数，不创建或修改源码文件：

```bash
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

cfg = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
params = Path(get_package_share_directory("dynamic_assembly")) / "config" / "params.yaml"
service = LaunchService()
service.include_launch_description(LaunchDescription([
    Node(
        package="dynamic_assembly", executable="mpc_planner_node",
        name="mpc_planner_node", output="screen",
        parameters=[str(params), cfg.robot_description,
                    cfg.robot_description_semantic, cfg.robot_description_kinematics,
                    {"use_sim_time": True}],
    )
]))
raise SystemExit(service.run())
PY
```

运行前确认机器人模型、控制器关节顺序、限位与场景安全条件匹配。本版本仍有下面列出的控制约束和数据时效性问题，分模块启动不等于修复了这些问题。

### 4. 真实相机输入与 RL 接口检查

真实相机模式下，替换上面的模拟输入入口，而不是额外增加一个目标源：

```bash
ros2 launch dynamic_assembly ekf_camera_aruco.launch.py --show-args
```

根据列出的参数设置设备、标定文件和实测外参。默认标记 ID 为 `0`、尺寸 `0.10` m、字典 `DICT_6X6_50`；实际使用必须与标记一致。默认相机文件路径及相机 TF 数值是开发机配置，不是通用标定。检查图像与 camera_info 的 frame，并验证该 frame 到 `world` 的 TF。`bringup.launch.py` 并没有自动包含相机专用 launch 中的全部 TF 配置。

仅观察 RL 接口的安全起点：

```bash
ros2 launch dynamic_assembly rl_residual.launch.py \
  enabled:=false policy_mode:=zero publish_corrected_to_controller:=false
ros2 topic echo /rl/observation --once
ros2 topic echo /rl/residual_action --once
```

38 维观测布局为 `q(6), dq(6), target_xyz(3), target_vxyz(3), path0_xyz(3), nominal_q0(6), nominal_dq0(6), flags(5)`。外部动作话题为 `/rl/external_action`，消息类型为 `Float64MultiArray`，6 个归一化动作按最大残差速度缩放。当前 flags 主要表示曾经收到过数据，不等价于新鲜度检查。

如需观察外部策略输出，可以使用 `enabled:=true policy_mode:=topic`，但仍保持 `publish_corrected_to_controller:=false`。**本快照不提供开启 RL 控制器输出的推荐流程**：必须先修复唯一发布者、数据过期、停止语义和修正轨迹约束等问题。默认关闭输出不能替代这些修复。

### 5. 数据采集、导出与绘图

在已确认运行的仿真中，可以录制以下话题；`/end_effector_path` 依赖外部轨迹工具。

```bash
ros2 bag record -o apf_local_avoidance_bag \
  /clock /joint_states /estimated_object_state \
  /predicted_target_trajectory /strategic_goal_path \
  /strategic_goal_path_marker /end_effector_path \
  /mpc_nominal_cmd /arm_controller/joint_trajectory
```

结束录制并正常关闭 bag 后，使用已经安装的导出脚本：

```bash
ros2 run dynamic_assembly export_apf_bag_to_csv.py \
  apf_local_avoidance_bag -o apf_local_avoidance_csv \
  --obstacle 0.32,0.05,0.04,0.04,0.06
```

`--obstacle` 的五个值分别为 X、Y、Z、半径和影响距离；支持重复指定。脚本也支持 `--storage-id auto|sqlite3|mcap`。导出的是预定义的路径、目标状态和标记数据，不会自动将 bag 中所有话题转换成 CSV。

在 MATLAB 中运行 `matlab/plot_apf_local_avoidance.m`，选择导出的 CSV 文件夹；脚本会寻找默认目录，找不到时弹出目录选择框。图像是实验可视化，不是自动完成的整臂碰撞验收。

MPC 支持启动时设置 `stats_csv_path`；默认空字符串，不写耗时 CSV。启用时指定可写路径，并注意当前同步 `flush()` 会影响控制线程耗时。参数更新没有完整动态回调，按重启生效处理。

### 6. 停止与诊断

- 纯仿真先在 Gazebo 暂停物理运动，再终止策略 / MPC 等发布者，最后关闭其余节点与仿真。仅 Ctrl-C 某个发布者不保证控制器立即撤销已接收的轨迹；节点暂停时 wall timer 仍可能运行。
- 本包的 `/emergency_stop` 属于未接入主链路的占位硬件节点，不能当作有效的机器人急停。真实设备必须采用其独立、已验证的停止措施。
- 启动报参数重复声明：检查 `assembly_system` 的自动声明与构造函数声明冲突，见下节。
- 模型加载失败：检查 AR4 描述 / SRDF / IKFast 依赖和参数是否传入，而不只是检查 topic 是否存在。
- 没有跟踪：依次检查 `/clock`、TF、`/joint_states`、目标估计、APF 路径、控制器状态和指令发布者数量。
- 控制指令发布者超过一个：先排除重复启动的 MPC 和 RL 输出，不通过调权重掩盖问题。
- 修改 YAML 后效果未变化：确认运行的是哪个 overlay、使用 `ros2 param get` 检查加载值，并重启对应节点。

## 已知问题与运行限制

本次仅同步原有算法代码并修正文档，以下问题 **尚未修复**：

1. `main.cpp` 开启 `automatically_declare_parameters_from_overrides(true)`，节点构造函数又无条件声明算法参数；带 YAML 的 `assembly_system` 路径存在重复声明异常风险。
2. APF 缓存旧目标并持续发布新路径，MPC 检查的是路径接收年龄，可能掩盖上游目标观测过期。
3. RL 仅充分检查外部动作年龄，缺少完整的关节 / 目标 / 名义轨迹新鲜度检查；重发旧名义轨迹和对回退轨迹叠加动作均需修复。
4. MPC 已直发控制器；开启 RL 直发会形成双发布者。RL 当前独立限幅速度、再给位置加 `residual * t`，限幅后位置 / 速度不一定一致；零残差也未必保持名义行为。末点加残差还可能违反控制器默认的零终端速度要求。
5. MPC 速度上限 2.0 rad/s、RL 修正速度上限 1.0 rad/s，与外部 AR4 限制未统一；缺少完整硬加速度约束和整臂碰撞检查。
6. 仿真时间与墙上时间混用，模拟目标按墙上时间运动；没有可靠的回合重置、同步步进和状态清空机制。
7. 5 ms MPC 配置周期与 50 ms 超时不匹配；每周期重建 OSQP，`warm_start` 没有实现跨周期求解器复用。未实测保证 200 Hz。
8. `max_tracking_error` 未实际执行检查；本地 `approach_height`、`max_force_norm` 等配置存在未生效项；yaw 终端权重应用范围也需检查。
9. 开发机外部 Gazebo 场景中的桌子和紫色障碍物只有 visual、没有 collision。这些外部文件未上传，也未在本次修改。
10. 参数声明、依赖元数据和测试覆盖仍需完善。仅有源码与文档静态检查，尚无本次同步版本的完整构建 / 仿真回归证据。

本次静态检查范围：主包 Python 文件语法、YAML / XML 解析、README 参数与路径核对、待上传源码与本地文件的一致性；未运行硬件控制、RL 训练或闭环性能测试。

## 实验展示与结果（待补充）

- **仿真演示：** 待加入视频或动图。
- **跟踪结果：** 待加入目标轨迹、末端轨迹和误差曲线；说明误差相对真实目标还是 APF 参考，以及是否扣除预期偏移。
- **规划效果：** 待加入静态障碍物场景和局部路径对比。
- **计算性能：** 待记录 CPU、预测步数、平均 / P95 求解耗时与超时比例。
- **RL 对比实验：** 待完成训练后，对比 MPC 基线、零残差与学习残差；目前不报告学习带来的性能提升。

所有结果应注明场景、目标速度、参数版本、采样方式和实验时长。未测量的指标不填写推测数值。

## 后续工作

1. 整理依赖版本和可复现操作说明，补充仿真视频及定量结果。
2. 构建 RL 训练环境、奖励函数与训练流程，接入策略推理并完成基线对比。
3. 完善 RL 输入时效性、修正轨迹约束检查与控制器输出切换。
4. 扩展动态障碍物处理和整臂碰撞验证。
5. 完成真实机械臂通信、反馈与控制联调，并开展实机验证。
6. 按课题进展扩展接触装配、力控或更完整的目标姿态处理。

## 贡献与许可

欢迎通过 Issue 反馈问题，通过 Pull Request 提交修改。提交时请说明改动目的、涉及模块和验证方法；修改由维护者审核后合并。

`package.xml` 当前声明 MIT。正式发布前应补充相应的 LICENSE 文件，并核对第三方代码、机器人模型与资源的来源及许可要求。
