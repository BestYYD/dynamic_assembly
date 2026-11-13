# Dynamic Assembly

基于 ROS 2 的六自由度机械臂动态目标跟踪研究代码：目标观测 → 平面状态估计与预测 → APF 局部参考 → MoveIt / OSQP 运动学 MPC → 强化学习智能体残差补正 →关节轨迹控制。


## 本次同步范围

2026-10-22 将本地 `dynamic_assembly` 主包同步到本仓库，补齐模拟 ArUco、RL 残差接口、数据导出和 MATLAB 绘图文件，并修正说明文档。未修改控制算法。


## 阅读入口

- [主包说明、环境、接口与调试步骤](dynamic_assembly/README.md)
- [已知问题与运行限制](dynamic_assembly/README.md#已知问题与运行限制)
- [备份说明与恢复方式](BACKUPS.md)
- [当前参数](dynamic_assembly/config/params.yaml)

主包 README 明确区分了启动文件的职责。不要在运行包含 KF / APF / MPC 的 `bringup.launch.py` 后重复启动这些节点。当前还存在参数重复声明、数据过期、安全约束和时间基准问题；本次源码同步并未修复它们。

## 仓库结构

```text
dynamic_assembly/                  # Git 仓库根目录
├── README.md
├── BACKUPS.md
├── .gitignore
└── dynamic_assembly/              # ROS 2 包
    ├── CMakeLists.txt
    ├── package.xml
    ├── src/
    ├── include/dynamic_assembly/
    ├── launch/
    ├── config/params.yaml
    ├── scripts/export_apf_bag_to_csv.py
    ├── matlab/plot_apf_local_avoidance.m
    └── README.md
```

## 环境与验证范围

开发机使用 WSL2 / Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。第一阶段低维残差控制研究不要求 Isaac Sim。本机已安装版本和外部依赖边界见主包说明。

本次检查包含源码同步一致性、Python 语法、YAML / XML 解析及文档核对；没有启动机械臂、执行完整构建 / 仿真回归或训练策略。不发布未测量的精度、实时频率和 RL 提升数据。

## 许可

主包 `package.xml` 声明 MIT，但独立 LICENSE 文件及权利信息仍需维护者补充。本次不擅自替换许可声明或给外部代码重新授权。
