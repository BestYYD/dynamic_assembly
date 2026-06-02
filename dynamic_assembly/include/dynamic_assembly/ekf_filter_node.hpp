#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include <Eigen/Dense>
#include <memory>
#include <string>
#include "dynamic_assembly/types.hpp"

namespace dynamic_assembly {

class EkfFilterNode : public rclcpp::Node {
public:
  explicit EkfFilterNode(const rclcpp::NodeOptions& options);

private:
  void observationCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
  bool transformObservationPose(
      const std_msgs::msg::Header& header,
      const geometry_msgs::msg::Pose& raw_pose,
      geometry_msgs::msg::Pose& tracking_pose);
  void predict();
  void update(const Eigen::Vector2d& z);
  void generatePredictedTrajectory();

  // EKF状态: [x, y, vx, vy]
  Eigen::Matrix<double, 4, 1> state_;
  Eigen::Matrix<double, 4, 4> P_;  // 状态协方差
  Eigen::Matrix<double, 4, 4> F_;  // 状态转移矩阵
  Eigen::Matrix<double, 2, 4> H_;  // 观测矩阵
  
  EkfConfig config_;
  bool initialized_ = false;
  double last_update_time_ = 0.0;

  int64_t target_marker_id_ = 0;
  double target_plane_z_ = 0.25;
  bool use_tf_transform_ = false;
  std::string aruco_markers_ = "/aruco_markers";
  std::string state_topic_ = "/estimated_object_state";
  std::string trajectory_topic_ = "/predicted_target_trajectory";
  std::string tracking_frame_ = "world";

  // ROS 接口
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr obs_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace dynamic_assembly
