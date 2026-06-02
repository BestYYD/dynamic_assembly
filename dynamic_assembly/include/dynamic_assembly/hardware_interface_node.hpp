#pragma once

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include "dynamic_assembly/types.hpp"

namespace dynamic_assembly {

class HardwareInterfaceNode : public rclcpp::Node {
public:
  explicit HardwareInterfaceNode(const rclcpp::NodeOptions& options);

private:
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void feedbackTimerCallback();
  void watchdogTimerCallback();

  // FPGA通信接口（模拟）
  bool writeToFpga(const trajectory_msgs::msg::JointTrajectory& traj);
  bool readFromFpga(RobotState& state);
  void emergencyStop();

  // 状态
  RobotState current_state_;
  double last_command_time_ = 0.0;
  double watchdog_timeout_ = 0.04;  // 40ms
  bool e_stop_triggered_ = false;

  // ROS接口
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace dynamic_assembly
