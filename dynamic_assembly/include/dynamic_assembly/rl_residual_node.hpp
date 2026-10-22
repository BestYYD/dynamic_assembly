#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <array>
#include <string>
#include <vector>

namespace dynamic_assembly {

// 残差强化学习框架节点：
// 默认只旁路监听 MPC 名义指令并发布观测/残差调试话题，不接管真实控制器。
class RlResidualNode : public rclcpp::Node {
public:
  explicit RlResidualNode(const rclcpp::NodeOptions& options);

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void targetPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void nominalCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void externalActionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void timerCallback();

  std_msgs::msg::Float64MultiArray buildObservation() const;
  std::array<double, 6> selectResidualAction(double now_time) const;
  trajectory_msgs::msg::JointTrajectory applyResidualToNominal(
      const trajectory_msgs::msg::JointTrajectory& nominal,
      const std::array<double, 6>& residual) const;
  std_msgs::msg::Float64MultiArray makeResidualMessage(
      const std::array<double, 6>& residual) const;

  double clampResidual(double value) const;
  double clampCorrectedVelocity(double value) const;
  double messageAge(double now_time, double stamp_time) const;
  static double stampToSeconds(const builtin_interfaces::msg::Time& stamp);
  static double durationToSeconds(const builtin_interfaces::msg::Duration& duration);

  std::vector<std::string> joint_names_ = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

  std::string joint_state_topic_ = "/joint_states";
  std::string target_state_topic_ = "/estimated_object_state";
  std::string target_path_topic_ = "/strategic_goal_path";
  std::string nominal_topic_ = "/mpc_nominal_cmd";
  std::string external_action_topic_ = "/rl/external_action";
  std::string observation_topic_ = "/rl/observation";
  std::string residual_action_topic_ = "/rl/residual_action";
  std::string corrected_topic_ = "/rl/corrected_cmd";
  std::string command_topic_ = "/arm_controller/joint_trajectory";
  std::string policy_mode_ = "zero";

  bool enabled_ = false;
  bool publish_corrected_to_controller_ = false;
  double period_ms_ = 50.0;
  double max_action_age_ = 0.2;
  double max_residual_velocity_ = 0.05;
  double max_corrected_velocity_ = 1.0;

  std::array<double, 6> joint_positions_{};
  std::array<double, 6> joint_velocities_{};
  std::array<double, 6> external_action_{};

  bool has_joint_state_ = false;
  bool has_target_state_ = false;
  bool has_target_path_ = false;
  bool has_nominal_command_ = false;
  bool has_external_action_ = false;

  double last_joint_state_time_ = 0.0;
  double last_target_state_time_ = 0.0;
  double last_target_path_time_ = 0.0;
  double last_nominal_time_ = 0.0;
  double last_external_action_time_ = 0.0;

  nav_msgs::msg::Odometry::SharedPtr target_state_;
  nav_msgs::msg::Path::SharedPtr target_path_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr nominal_command_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr nominal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr external_action_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr observation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr residual_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr corrected_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dynamic_assembly
