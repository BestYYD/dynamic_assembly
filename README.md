# Dynamic Assembly

基于 ROS2 的六自由度机械臂动态目标跟踪与预测控制项目，面向动态装配场景，包含目标状态估计、APF 局部路径规划、运动学 MPC 控制以及强化学习残差控制接口。

## 项目内容

- 基于 ArUco 的动态目标检测与状态估计
- 基于 TF2 的坐标变换
- 基于 APF 的局部路径规划
- 基于 MoveIt 和 OSQP 的运动学 MPC
- ROS2 话题通信与关节轨迹控制
- 强化学习残差控制扩展接口
- AR4 机械臂仿真验证

目前已完成仿真闭环验证，实机验证和强化学习策略训练仍在进行中。

## 仓库结构

```text
workspace_win/
├── README.md
└── dynamic_assembly/
    ├── src/
    ├── include/
    ├── launch/
    ├── config/
    ├── scripts/
    ├── matlab/
    └── README.md