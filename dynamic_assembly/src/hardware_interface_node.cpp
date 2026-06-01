#include "dynamic_assembly/hardware_interface_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace dynamic_assembly {

HardwareInterfaceNode::HardwareInterfaceNode(const rclcpp::NodeOptions& options)
    : Node("hardware_interface_node", options) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", true);
  }
  if (!get_parameter("use_sim_time").as_bool()) {
    set_parameter(rclcpp::Parameter("use_sim_time", true));
  }
  
  // 声明参数
  declare_parameter("watchdog_timeout", 0.04);
  declare_parameter("feedback_rate", 100.0);
  
  watchdog_timeout_ = get_parameter("watchdog_timeout").as_double();
  double feedback_rate = get_parameter("feedback_rate").as_double();

  // 初始化状态
  current_state_.joint_positions.setZero();
  current_state_.joint_velocities.setZero();
  current_state_.ee_position = Eigen::Vector3d(0.4, 0.0, 0.3);
  current_state_.ee_orientation = Eigen::Quaterniond::Identity();

  // 订阅轨迹指令
  traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/micro_trajectory_command", 10,
      std::bind(&HardwareInterfaceNode::trajectoryCallback, this, std::placeholders::_1));

  // 发布机器人状态
  state_pub_ = create_publisher<nav_msgs::msg::Odometry>("/robot_state", 10);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);

  // 状态反馈定时器 (100Hz)
  feedback_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / feedback_rate)),
      std::bind(&HardwareInterfaceNode::feedbackTimerCallback, this));

  // 看门狗定时器 (200Hz检查)
  watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&HardwareInterfaceNode::watchdogTimerCallback, this));

  last_command_time_ = now_sec();

  RCLCPP_INFO(get_logger(), "Hardware Interface Node initialized");
}

void HardwareInterfaceNode::trajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  
  if (e_stop_triggered_) {
    RCLCPP_WARN(get_logger(), "E-Stop active, ignoring trajectory command");
    return;
  }

  // 验证轨迹合法性
  if (msg->points.empty()) {
    RCLCPP_WARN(get_logger(), "Empty trajectory received");
    return;
  }

  // 关节限位检查 (6轴机械臂)
  const double joint_limits[] = {2.96, 2.09, 2.96, 2.09, 2.96, 3.14};  // 弧度
  for (const auto& point : msg->points) {
    for (size_t i = 0; i < point.positions.size() && i < 6; ++i) {
      if (std::abs(point.positions[i]) > joint_limits[i]) {
        RCLCPP_ERROR(get_logger(), "Joint %zu exceeds limit: %.3f > %.3f", 
                     i, point.positions[i], joint_limits[i]);
        emergencyStop();
        return;
      }
    }
  }

  // 写入FPGA
  if (writeToFpga(*msg)) {
    last_command_time_ = now_sec();
  }
}

void HardwareInterfaceNode::feedbackTimerCallback() {
  // 从FPGA读取状态
  readFromFpga(current_state_);

  // 发布状态
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now();
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "end_effector";

  odom_msg.pose.pose.position.x = current_state_.ee_position.x();
  odom_msg.pose.pose.position.y = current_state_.ee_position.y();
  odom_msg.pose.pose.position.z = current_state_.ee_position.z();

  odom_msg.pose.pose.orientation.w = current_state_.ee_orientation.w();
  odom_msg.pose.pose.orientation.x = current_state_.ee_orientation.x();
  odom_msg.pose.pose.orientation.y = current_state_.ee_orientation.y();
  odom_msg.pose.pose.orientation.z = current_state_.ee_orientation.z();

  state_pub_->publish(odom_msg);
}

void HardwareInterfaceNode::watchdogTimerCallback() {
  double current_time = now_sec();
  
  if (!e_stop_triggered_ && (current_time - last_command_time_) > watchdog_timeout_) {
    RCLCPP_ERROR(get_logger(), "Watchdog timeout! No command for %.1fms", 
                 (current_time - last_command_time_) * 1000);
    emergencyStop();
  }
}

bool HardwareInterfaceNode::writeToFpga(const trajectory_msgs::msg::JointTrajectory& traj) {
  // ========== FPGA通信实现 ==========
  // 实际项目中这里实现：
  // 1. 共享内存写入
  // 2. PCIe通信
  // 3. 以太网UDP发送
  
  // 模拟写入成功
  RCLCPP_DEBUG(get_logger(), "Written trajectory with %zu points to FPGA", 
               traj.points.size());
  return true;
}

bool HardwareInterfaceNode::readFromFpga(RobotState& state) {
  // ========== FPGA状态读取 ==========
  // 实际项目中这里实现：
  // 1. 共享内存读取编码器值
  // 2. 正运动学计算末端位姿
  
  // 模拟：添加微小变化
  static double t = 0.0;
  t += 0.01;
  state.ee_position.x() = 0.4 + 0.01 * std::sin(t);
  state.timestamp = now_sec();
  
  return true;
}

void HardwareInterfaceNode::emergencyStop() {
  e_stop_triggered_ = true;
  
  // 发布紧急停止信号
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub_->publish(estop_msg);

  RCLCPP_FATAL(get_logger(), "EMERGENCY STOP TRIGGERED!");

  // 实际项目中这里应该：
  // 1. 向FPGA发送刹车指令
  // 2. 切断伺服使能
  // 3. 记录日志
}

}  // namespace dynamic_assembly

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_assembly::HardwareInterfaceNode)
