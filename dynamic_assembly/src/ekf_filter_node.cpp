#include "dynamic_assembly/ekf_filter_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

namespace dynamic_assembly {

EkfFilterNode::EkfFilterNode(const rclcpp::NodeOptions& options) : Node("ekf_filter_node", options) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", true);
  }
  if (!get_parameter("use_sim_time").as_bool()) {
    set_parameter(rclcpp::Parameter("use_sim_time", true));
  }
  
  // 声明参数
  declare_parameter("prediction_steps", 50);
  declare_parameter("prediction_dt", 0.02);
  declare_parameter("aruco_markers", aruco_markers_);
  declare_parameter("state_topic", state_topic_);
  declare_parameter("trajectory_topic", trajectory_topic_);
  declare_parameter("target_marker_id", target_marker_id_);
  declare_parameter("target_plane_z", target_plane_z_);
  declare_parameter("tracking_frame", tracking_frame_);
  declare_parameter("use_tf_transform", use_tf_transform_);

  config_.prediction_steps = get_parameter("prediction_steps").as_int();
  config_.prediction_dt = get_parameter("prediction_dt").as_double();
  aruco_markers_ = get_parameter("aruco_markers").as_string();
  state_topic_ = get_parameter("state_topic").as_string();
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  target_marker_id_ = get_parameter("target_marker_id").as_int();
  target_plane_z_ = get_parameter("target_plane_z").as_double();
  tracking_frame_ = get_parameter("tracking_frame").as_string();
  use_tf_transform_ = get_parameter("use_tf_transform").as_bool();

  // 初始化EKF矩阵
  state_.setZero();
  P_.setIdentity();
  P_ *= 1.0;
  
  // 状态转移矩阵 (恒速模型)
  F_.setIdentity();
  // F会在predict中根据dt动态更新

  // 观测矩阵 (只观测位置)
  H_.setZero();
  H_(0, 0) = H_(1, 1)  = 1.0;

  // 过程噪声和观测噪声
  config_.Q.setIdentity();
  config_.Q *= 0.1;
  config_.R.setIdentity();
  config_.R *= 0.01;

  if (use_tf_transform_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // 订阅 Aruco 检测结果，输出给 APF 使用的当前估计状态和预测轨迹。
  obs_sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      aruco_markers_, 10,
      std::bind(&EkfFilterNode::observationCallback, this, std::placeholders::_1));

  state_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      state_topic_, 10);

  trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      trajectory_topic_, 10);

  RCLCPP_INFO(
      get_logger(),
      "EKF initialized: marker topic=%s, target id=%ld, output frame=%s",
      aruco_markers_.c_str(), target_marker_id_, tracking_frame_.c_str());
}

void EkfFilterNode::observationCallback(
    const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
  
  const size_t marker_count = std::min(msg->marker_ids.size(), msg->poses.size());
  if (marker_count == 0) {
    return;
  }

  if (msg->marker_ids.size() != msg->poses.size()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Aruco marker id count (%zu) does not match pose count (%zu)",
        msg->marker_ids.size(), msg->poses.size());
  }

  // 寻找指定 ID 的标记，默认 ID=0。
  int target_index = -1;
  for (size_t i = 0; i < marker_count; ++i) {
    if (msg->marker_ids[i] == target_marker_id_) {
      target_index = static_cast<int>(i);
      break;
    }
  }
  
  // 没找到目标 marker 时不更新 EKF，保持上一帧预测。
  if (target_index == -1) {
    return;
  }

  geometry_msgs::msg::Pose tracking_pose;
  if (!transformObservationPose(msg->header, msg->poses[static_cast<size_t>(target_index)], tracking_pose)) {
    return;
  }

  double current_time = now_sec();
  
  // 目标只在二维平面运动，EKF 状态只估计 x/y 和 vx/vy。
  Eigen::Vector2d z(tracking_pose.position.x, tracking_pose.position.y);

  if (!initialized_) {
    state_.head<2>() = z;
    state_.tail<2>().setZero();
    last_update_time_ = current_time;
    initialized_ = true;
    update(z);
    generatePredictedTrajectory();
    return;
  }

  // 计算dt并执行预测
  double dt = current_time - last_update_time_;
  if (dt > 0.001) {
    F_.setIdentity();
    F_(0, 2) = F_(1, 3) = dt;
    predict();
  }

  // 执行更新
  update(z);
  last_update_time_ = current_time;

  // 生成并发布预测轨迹
  generatePredictedTrajectory();
}

bool EkfFilterNode::transformObservationPose(
    const std_msgs::msg::Header& header,
    const geometry_msgs::msg::Pose& raw_pose,
    geometry_msgs::msg::Pose& tracking_pose) {
  if (!use_tf_transform_ || header.frame_id.empty() || header.frame_id == tracking_frame_) {
    tracking_pose = raw_pose;
    return true;
  }

  if (!tf_buffer_) {
    return false;
  }

  geometry_msgs::msg::PoseStamped input;
  input.header = header;
  input.pose = raw_pose;

  try {
    const auto output = tf_buffer_->transform(
        input, tracking_frame_, tf2::durationFromSec(0.02));
    tracking_pose = output.pose;
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform Aruco pose from '%s' to '%s': %s",
        header.frame_id.c_str(), tracking_frame_.c_str(), ex.what());
    return false;
  }
}

void EkfFilterNode::predict() {
  state_ = F_ * state_; //预测状态
  P_ = F_ * P_ * F_.transpose() + config_.Q; //预测的不确定性
}

void EkfFilterNode::update(const Eigen::Vector2d& z) {
  // 卡尔曼增益K
  Eigen::Matrix2d S = H_ * P_ * H_.transpose() + config_.R;
  Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();

  // 状态更新
  Eigen::Vector2d y = z - H_ * state_;
  state_ = state_ + K * y;
  
  // 协方差更新
  Eigen::Matrix<double, 4, 4> I4 = Eigen::Matrix<double, 4, 4>::Identity();
  P_ = (I4 - K * H_) * P_;

  // 发布当前状态估计
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now();
  odom_msg.header.frame_id = tracking_frame_;
  odom_msg.pose.pose.position.x = state_(0);
  odom_msg.pose.pose.position.y = state_(1);
  odom_msg.pose.pose.position.z = target_plane_z_;  // 目标在固定高度的二维平面上

  odom_msg.twist.twist.linear.x = state_(2);
  odom_msg.twist.twist.linear.y = state_(3);
  odom_msg.twist.twist.linear.z = 0.0;

  double vx = state_(2);
  double vy = state_(3);
  double speed = std::sqrt(vx * vx + vy * vy);
  // 只有当速度大于一定阈值时才更新朝向，防止静止时箭头乱跳
  if (speed > 0.05) { 
    double yaw = std::atan2(vy, vx);
    // 将 Yaw 转换为四元数 (z = sin(yaw/2), w = cos(yaw/2))
    odom_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  } else {
    // 静止时保持默认 (或者你可以保存上一次的朝向)
    odom_msg.pose.pose.orientation.w = 1.0; 
  }

  
  // 填充协方差（只填位置部分）
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      odom_msg.pose.covariance[i * 6 + j] = P_(i, j);

  state_pub_->publish(odom_msg);
}

void EkfFilterNode::generatePredictedTrajectory() {
  auto path_msg = nav_msgs::msg::Path();
  path_msg.header.stamp = now();
  path_msg.header.frame_id = tracking_frame_;

  // 从当前状态向前外推
  Eigen::Matrix<double, 4, 1> pred_state = state_;
  Eigen::Matrix<double, 4, 4> F_pred = Eigen::Matrix<double, 4, 4>::Identity();
  F_pred(0, 2) = F_pred(1, 3) = config_.prediction_dt;

  for (int i = 0; i < config_.prediction_steps; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Time(now()) + rclcpp::Duration::from_seconds(i * config_.prediction_dt);
    pose.header.frame_id = tracking_frame_;
    pose.pose.position.x = pred_state(0);
    pose.pose.position.y = pred_state(1);
    pose.pose.position.z = target_plane_z_;  // 目标在固定高度的二维平面上

    // [新增] 计算预测点的速度朝向
    double p_vx = pred_state(2);
    double p_vy = pred_state(3);
    double p_speed = std::sqrt(p_vx * p_vx + p_vy * p_vy);

    if (p_speed > 0.02) {
        double p_yaw = std::atan2(p_vy, p_vx);
        pose.pose.orientation.z = std::sin(p_yaw / 2.0);
        pose.pose.orientation.w = std::cos(p_yaw / 2.0);
    } else {
        pose.pose.orientation.w = 1.0;
    }

    path_msg.poses.push_back(pose);

    pred_state = F_pred * pred_state;
  }

  trajectory_pub_->publish(path_msg);
}

}  // namespace dynamic_assembly

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_assembly::EkfFilterNode)
