# 动态目标装配系统 - ROS2框架

## 项目概述

基于ROS2的六自由度机械臂动态目标装配系统，采用分层规划架构：

```
┌─────────────────────────────────────────────────────────────────┐
│                        X86 平台 (ROS2)                          │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐    │
│  │ EKF Filter   │────▶│ Strategic    │────▶│ Tactical     │    │
│  │ (状态估计)    │     │ Planner(PSO) │     │ Planner(MPC) │    │
│  │              │     │ 500ms        │     │ 20ms         │    │
│  └──────────────┘     └──────────────┘     └──────────────┘    │
│         ▲                                          │            │
│         │                                          ▼            │
│  ┌──────┴───────────────────────────────────────────┐          │
│  │            Hardware Interface Node               │          │
│  │              (FPGA通信 + 看门狗)                  │          │
│  └─────────────────────┬────────────────────────────┘          │
└────────────────────────┼────────────────────────────────────────┘
                         ▼
                 ┌───────────────┐
                 │     FPGA      │
                 │  (运动控制器)  │
                 └───────────────┘
```

## 目录结构

```
dynamic_assembly/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── params.yaml              # 所有节点参数
├── include/dynamic_assembly/
│   ├── types.hpp                # 公共类型定义
│   ├── ekf_filter_node.hpp
│   ├── apf_planner_node.hpp
│   ├── mpc_planner_node.hpp
│   └── hardware_interface_node.hpp
├── src/
│   ├── ekf_filter_node.cpp      # EKF滤波与预测
│   ├── apf_planner_node.cpp       # APF前瞻点规划
│   ├── mpc_planner_node.cpp       # MPC局部跟踪控制
│   └── hardware_interface_node.cpp # FPGA通信
└── launch/
    └── bringup.launch.py        # 系统启动文件
```

## 话题通信

| 话题名称 | 消息类型 | 发布者 | 订阅者 |
|---------|---------|--------|--------|
| `/detected_object_pose` | PoseStamped | 视觉系统 | EKF |
| `/estimated_object_state` | Odometry | EKF | - |
| `/predicted_target_trajectory` | Path | EKF | PSO |
| `/strategic_goal_path` | Path | PSO | MPC |
| `/robot_state` | Odometry | HW | MPC, PSO |
| `/micro_trajectory_command` | JointTrajectory | MPC | HW |
| `/emergency_stop` | Bool | HW | 全局 |

## 编译与运行

```bash
# 编译
cd dynamic_assembly_ws
colcon build --packages-select dynamic_assembly

# 运行
source install/setup.bash
ros2 launch dynamic_assembly bringup.launch.py
```

## 关键设计点

### 1. 实时性保障
- MPC和硬件接口运行在隔离的实时容器中
- 看门狗超时机制（40ms无指令触发急停）

### 2. 降级策略
- MPC求解超时（18ms）自动切换刹车轨迹
- 数据过期（50ms）执行安全动作

### 3. 安全机制
- 关节限位硬件级检查
- 跟踪误差监控
- 通信超时检测

## 待实现功能

1. **MPC求解器**：集成OSQP/qpOASES
2. **逆运动学**：集成KDL或自定义IK
3. **FPGA通信**：实现共享内存/PCIe接口
4. **障碍物处理**：动态障碍物预测与避障
