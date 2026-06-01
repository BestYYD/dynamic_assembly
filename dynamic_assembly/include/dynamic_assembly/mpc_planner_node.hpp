#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>
#include <cstddef>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include "dynamic_assembly/types.hpp"

namespace dynamic_assembly {

// 笛卡尔空间运动学 LTV-MPC：接收 APF 前瞻点，直接在末端位姿空间里做预测跟踪。
class MpcPlannerNode : public rclcpp::Node {
public:
  explicit MpcPlannerNode(const rclcpp::NodeOptions& options);
  void initialize_MPC();

private:
  // 输入和控制周期回调。
  void goalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlTimerCallback();

  // MPC 主流程：构造笛卡尔参考、线性化运动学模型、求解关节速度 QP。
  bool solveMpc(trajectory_msgs::msg::JointTrajectory& output);
  struct PlanarPoseReference {
    Eigen::Vector3d position;
    double yaw = 0.0;
  };

  bool buildReferencePlanarPoseTrajectory(
      std::vector<PlanarPoseReference>& references,
      double current_yaw) const;
  bool buildLinearizedKinematicModel(
      const std::vector<PlanarPoseReference>& references,
      std::vector<Eigen::Matrix<double, 6, 6>>& jacobians);
  bool computePlanarPoseJacobian(
      const Eigen::Matrix<double, 6, 1>& q,
      Eigen::Matrix<double, 6, 6>& jacobian,
      Eigen::Vector3d* ee_position,
      Eigen::Vector3d* tool_z_axis,
      double* ee_yaw);
  Eigen::Matrix<double, 6, 1> dampedLeastSquaresVelocity(
      const Eigen::Matrix<double, 6, 6>& jacobian,
      const Eigen::Matrix<double, 6, 1>& planar_pose_error) const;
  bool solveCartesianLtvMpc(
      const Eigen::Matrix<double, 6, 1>& x0,
      const Eigen::Matrix<double, 6, 1>& q0,
      const Eigen::Matrix<double, 6, 1>& u_previous,
      const std::vector<PlanarPoseReference>& references,
      const std::vector<Eigen::Matrix<double, 6, 6>>& jacobians,
      std::vector<Eigen::Matrix<double, 6, 1>>& optimal_velocities);
  std::size_t targetPoseIndexForStep(int step) const;
  void fillJointTrajectoryPoint(
      const Eigen::Matrix<double, 6, 1>& positions,
      const Eigen::Matrix<double, 6, 1>& velocities,
      double time_from_start,
      trajectory_msgs::msg::JointTrajectoryPoint& point) const;
  Eigen::Matrix<double, 6, 1> clampJointVelocity(const Eigen::VectorXd& q_dot) const;
  void clampToJointLimits(Eigen::Matrix<double, 6, 1>& q) const;
  
  // 降级策略：数据过期或求解失败时发送保持当前位置的零速度轨迹。
  void executeFallback();

  MpcConfig config_;
  nav_msgs::msg::Path::SharedPtr goal_path_;
  RobotState robot_state_;
  trajectory_msgs::msg::JointTrajectory last_trajectory_;
  bool has_last_trajectory_ = false;

  // 状态标志和数据新鲜度限制。
  double last_goal_time_ = 0.0;
  double last_joint_state_time_ = 0.0;
  double max_data_age_ = 0.15;
  bool has_joint_state_ = false;

  // ROS 接口。
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr nominal_traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 机械臂关节和控制话题配置。
  std::vector<std::string> joint_names_ = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  std::string command_topic_ = "/arm_controller/joint_trajectory";
  std::string nominal_topic_ = "/mpc_nominal_cmd";
  std::string goal_path_topic_ = "/strategic_goal_path";
  std::string joint_state_topic_ = "/joint_states";
  std::string tip_link_name_ = "link_6";

  Eigen::Matrix<double, 6, 1> current_joint_positions_;
  Eigen::Matrix<double, 6, 1> current_joint_velocities_;

  // QP/OSQP 参数：笛卡尔跟踪权重、正则化、速度上限和求解精度。
  double approach_height_ = 0.25;
  double cartesian_z_weight_ = 1.0;
  double linearization_damping_ = 0.05;
  double qp_regularization_ = 1.0e-6;
  double max_joint_velocity_ = 0.25;
  double terminal_weight_ = 2.0;
  double control_delta_weight_ = 0.05;
  bool track_planar_yaw_ = true;
  double z_axis_alignment_weight_ = 2.0;
  double yaw_tracking_weight_ = 0.5;
  double yaw_terminal_weight_ = 1.0;
  int osqp_max_iterations_ = 400;
  double osqp_eps_abs_ = 1.0e-3;
  double osqp_eps_rel_ = 1.0e-3;

  Eigen::Matrix<double, 6, 1> joint_lower_limits_;
  Eigen::Matrix<double, 6, 1> joint_upper_limits_;

  // MoveIt 运动学对象，用 FK/Jacobian 在关节速度和末端速度之间转换。
  std::shared_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  moveit::core::RobotModelPtr moveit_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
  bool mpc_initialized_ = false;
  bool has_locked_yaw_reference_ = false;
  double locked_yaw_reference_ = 0.0;
};

}  // namespace dynamic_assembly
