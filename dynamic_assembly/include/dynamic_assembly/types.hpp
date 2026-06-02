#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <cmath>
#include <algorithm> 

namespace dynamic_assembly {

// 系统状态枚举
enum class SystemState {
  INIT,
  CALIBRATING,
  IDLE,
  TRACKING,
  APPROACHING,
  ASSEMBLY,
  ERROR,
  EMERGENCY_STOP
};

// 机器人状态结构 (6轴机械臂，3D空间)
struct RobotState {
  Eigen::Matrix<double, 6, 1> joint_positions;
  Eigen::Matrix<double, 6, 1> joint_velocities;
  Eigen::Vector3d ee_position;       // 末端位置 (3D)
  Eigen::Quaterniond ee_orientation; // 末端姿态
  double timestamp = 0.0;
};

// 目标状态结构（2D平面运动，含协方差）
struct TargetState {
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  Eigen::Matrix2d covariance;  // 位置协方差，表示不确定性
  double timestamp;
};

// MPC配置参数
struct MpcConfig {
  int horizon = 20;              // 预测步数
  double dt = 0.02;              // 控制周期 20ms
  double timeout_ms = 18.0;      // 求解超时
  double tracking_weight = 1.0;
  double control_weight = 0.01;
  double obstacle_weight = 10.0;
};

// PSO配置参数
struct PsoConfig {
  int num_particles = 30;
  int max_iterations = 50;
  double w = 0.7;   // 惯性权重
  double c1 = 1.5;  // 认知系数
  double c2 = 1.5;  // 社会系数
  double period_ms = 500.0;  // 优化周期
};

// EKF配置参数
struct EkfConfig {
  int prediction_steps = 50;  // 预测步数
  double prediction_dt = 0.02;
  Eigen::Matrix<double, 4, 4> Q;  // 过程噪声
  Eigen::Matrix<double, 2, 2> R;  // 观测噪声
};

// 获取当前时间戳（秒）
inline double now_sec() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace dynamic_assembly
