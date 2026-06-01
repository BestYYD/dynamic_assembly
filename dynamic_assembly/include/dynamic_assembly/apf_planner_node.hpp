#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <string>
#include <vector>
#include "dynamic_assembly/types.hpp"

namespace dynamic_assembly {

// 人工势场规划节点：把二维平面上的移动目标转换为机械臂末端可跟踪的三维短路径。
class ApfPlannerNode : public rclcpp::Node {
public:
  explicit ApfPlannerNode(const rclcpp::NodeOptions& options);

  void initialize_APF();
  void initialize_PSO();

private:
  struct Obstacle {
    Eigen::Vector3d center;
    double radius = 0.0;
  };

  // 输入回调：目标轨迹/目标状态提供平面目标，关节状态用于 FK 计算末端当前位置。
  void targetTrajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void targetStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void planningTimerCallback();

  // APF 主流程：更新末端状态、选目标点、计算势场力、发布短时域路径。
  bool updateEndEffectorState();
  bool selectTargetPointAtTime(double prediction_time, Eigen::Vector3d& target_point) const;
  bool buildLookaheadPath(std::vector<Eigen::Vector3d>& lookahead_path) const;
  Eigen::Vector3d computeAttractiveForce(
      const Eigen::Vector3d& from_point,
      const Eigen::Vector3d& target_point) const;
  Eigen::Vector3d computeRepulsiveForce(const Eigen::Vector3d& from_point) const;
  Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d& value, double max_norm) const;
  Eigen::Vector3d clampWorkspace(const Eigen::Vector3d& point) const;
  void publishLookaheadPath(const std::vector<Eigen::Vector3d>& lookahead_path);

  std::vector<Obstacle> loadObstaclesFromParameters() const;

  std::vector<std::string> joint_names_ = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

  // 话题和坐标系参数，默认保持与当前仿真/MoveIt 配置一致。
  std::string frame_id_ = "world";
  std::string tip_link_name_ = "link_6";
  std::string joint_state_topic_ = "/joint_states";
  std::string target_path_topic_ = "/predicted_target_trajectory";
  std::string target_state_topic_ = "/estimated_object_state";
  std::string lookahead_topic_ = "/apf/lookahead_point";
  std::string legacy_path_topic_ = "/strategic_goal_path";
  std::string marker_topic_ = "/strategic_goal_path_marker";

  // APF 参数：目标在 XY 平面运动，target_plane_z_ 决定映射到三维空间后的跟踪高度。
  double period_ms_ = 50.0;
  double target_prediction_time_ = 0.2;
  double target_plane_z_ = 0.25;
  double attractive_gain_ = 1.0;
  double repulsive_gain_ = 0.15;
  double obstacle_influence_distance_ = 0.25;
  double obstacle_min_distance_ = 0.03;
  double lookahead_distance_ = 0.08;
  double max_force_norm_ = 1.0;
  double min_force_norm_ = 1.0e-4;
  double filter_alpha_ = 0.35;
  int output_path_points_ = 10;
  double output_path_dt_ = 0.1;
  bool publish_legacy_path_ = true;

  Eigen::Vector3d workspace_min_ = Eigen::Vector3d(-1.0, -1.0, 0.02);
  Eigen::Vector3d workspace_max_ = Eigen::Vector3d(1.0, 1.0, 1.0);

  // 最近一次收到的目标信息，轨迹优先；没有轨迹时退回到单点状态。
  nav_msgs::msg::Path::SharedPtr target_trajectory_;
  nav_msgs::msg::Odometry::SharedPtr target_state_;

  RobotState robot_state_;
  Eigen::Matrix<double, 6, 1> current_joint_positions_;
  bool has_joint_state_ = false;
  bool has_filtered_lookahead_ = false;
  bool apf_initialized_ = false;
  Eigen::Vector3d filtered_lookahead_;

  std::vector<Obstacle> obstacles_;

  // ROS 通信对象。
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_traj_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr legacy_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // MoveIt 运动学对象，用来由关节角实时计算末端位姿。
  std::shared_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  moveit::core::RobotModelPtr moveit_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  const moveit::core::JointModelGroup* joint_model_group_ = nullptr;
};

}  // namespace dynamic_assembly
