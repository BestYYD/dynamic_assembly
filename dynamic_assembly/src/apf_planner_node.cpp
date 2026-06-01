#include "dynamic_assembly/apf_planner_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dynamic_assembly {

namespace {

// ROS 时间戳统一转成秒，便于按预测时间索引目标轨迹。
double stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

// 参数文件里的三维向量必须是 [x, y, z]，长度不对时保留默认值。
Eigen::Vector3d vectorFromParameter(
    const std::vector<double>& values,
    const Eigen::Vector3d& fallback) {
  if (values.size() != 3) {
    return fallback;
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

// 优先使用消息时间戳选择预测点；没有有效时间戳时按固定周期估算索引。
size_t targetIndexForPrediction(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    double prediction_time,
    double fallback_dt) {
  if (poses.size() <= 1) {
    return 0;
  }

  const double first_stamp = stampToSeconds(poses.front().header.stamp);
  const double last_stamp = stampToSeconds(poses.back().header.stamp);
  if (last_stamp > first_stamp) {
    const double query_time = first_stamp + prediction_time;
    for (size_t i = 0; i < poses.size(); ++i) {
      if (stampToSeconds(poses[i].header.stamp) >= query_time) {
        return i;
      }
    }
    return poses.size() - 1;
  }

  const auto fallback_index = static_cast<size_t>(
      std::max(1.0, prediction_time / std::max(0.001, fallback_dt)));
  return std::min(poses.size() - 1, fallback_index);
}

}  // namespace

ApfPlannerNode::ApfPlannerNode(const rclcpp::NodeOptions& options)
    : Node("apf_planner_node", options) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", true);
  }
  if (!get_parameter("use_sim_time").as_bool()) {
    set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  // 所有 APF 参数都从 ROS 参数读取，便于后续在 launch/params.yaml 中调参。
  declare_parameter("period_ms", period_ms_);
  declare_parameter("target_prediction_time", target_prediction_time_);
  declare_parameter("target_plane_z", target_plane_z_);
  declare_parameter("attractive_gain", attractive_gain_);
  declare_parameter("repulsive_gain", repulsive_gain_);
  declare_parameter("obstacle_influence_distance", obstacle_influence_distance_);
  declare_parameter("obstacle_min_distance", obstacle_min_distance_);
  declare_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter("max_force_norm", max_force_norm_);
  declare_parameter("min_force_norm", min_force_norm_);
  declare_parameter("filter_alpha", filter_alpha_);
  declare_parameter("output_path_points", output_path_points_);
  declare_parameter("output_path_dt", output_path_dt_);
  declare_parameter("publish_legacy_path", publish_legacy_path_);
  declare_parameter("frame_id", frame_id_);
  declare_parameter("tip_link_name", tip_link_name_);
  declare_parameter("joint_state_topic", joint_state_topic_);
  declare_parameter("target_path_topic", target_path_topic_);
  declare_parameter("target_state_topic", target_state_topic_);
  declare_parameter("lookahead_topic", lookahead_topic_);
  declare_parameter("legacy_path_topic", legacy_path_topic_);
  declare_parameter("marker_topic", marker_topic_);
  
  declare_parameter<std::vector<std::string>>("joint_names", joint_names_);
  declare_parameter<std::vector<double>>("workspace_min", {-1.0, -1.0, 0.02});
  declare_parameter<std::vector<double>>("workspace_max", {1.0, 1.0, 1.0});
  declare_parameter<std::vector<double>>("obstacle_centers", std::vector<double>{});
  declare_parameter<std::vector<double>>("obstacle_radii", std::vector<double>{});

  period_ms_ = get_parameter("period_ms").as_double();
  target_prediction_time_ = get_parameter("target_prediction_time").as_double();
  target_plane_z_ = get_parameter("target_plane_z").as_double();
  attractive_gain_ = get_parameter("attractive_gain").as_double();
  repulsive_gain_ = get_parameter("repulsive_gain").as_double();
  obstacle_influence_distance_ = get_parameter("obstacle_influence_distance").as_double();
  obstacle_min_distance_ = get_parameter("obstacle_min_distance").as_double();
  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  max_force_norm_ = get_parameter("max_force_norm").as_double();
  min_force_norm_ = get_parameter("min_force_norm").as_double();
  filter_alpha_ = std::clamp(get_parameter("filter_alpha").as_double(), 0.0, 1.0);
  output_path_points_ = std::max(1, static_cast<int>(get_parameter("output_path_points").as_int()));
  output_path_dt_ = std::max(0.001, get_parameter("output_path_dt").as_double());
  publish_legacy_path_ = get_parameter("publish_legacy_path").as_bool();
  frame_id_ = get_parameter("frame_id").as_string();
  tip_link_name_ = get_parameter("tip_link_name").as_string();
  joint_state_topic_ = get_parameter("joint_state_topic").as_string();
  target_path_topic_ = get_parameter("target_path_topic").as_string();
  target_state_topic_ = get_parameter("target_state_topic").as_string();
  lookahead_topic_ = get_parameter("lookahead_topic").as_string();
  legacy_path_topic_ = get_parameter("legacy_path_topic").as_string();
  marker_topic_ = get_parameter("marker_topic").as_string();
  joint_names_ = get_parameter("joint_names").as_string_array();
  workspace_min_ = vectorFromParameter(get_parameter("workspace_min").as_double_array(), workspace_min_);
  workspace_max_ = vectorFromParameter(get_parameter("workspace_max").as_double_array(), workspace_max_);
  obstacles_ = loadObstaclesFromParameters();

  current_joint_positions_.setZero();
  robot_state_.ee_position = Eigen::Vector3d::Zero();
  robot_state_.ee_orientation = Eigen::Quaterniond::Identity();

  target_traj_sub_ = create_subscription<nav_msgs::msg::Path>(
      target_path_topic_, 10,
      std::bind(&ApfPlannerNode::targetTrajectoryCallback, this, std::placeholders::_1));

  target_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      target_state_topic_, 10,
      std::bind(&ApfPlannerNode::targetStateCallback, this, std::placeholders::_1));

  // JointState 用来做正运动学，APF 本身只在末端任务空间里计算方向。
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ApfPlannerNode::jointStateCallback, this, std::placeholders::_1));

  lookahead_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(lookahead_topic_, 10);
  legacy_path_pub_ = create_publisher<nav_msgs::msg::Path>(legacy_path_topic_, 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

  timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms_)),
      std::bind(&ApfPlannerNode::planningTimerCallback, this));

  RCLCPP_INFO(
      get_logger(),
      "APF planner initialized: %.1fms period, %d path points, dt=%.3fs",
      period_ms_, output_path_points_, output_path_dt_);
}

void ApfPlannerNode::initialize_PSO() {
  initialize_APF();
}

void ApfPlannerNode::initialize_APF() {
  if (apf_initialized_) {
    return;
  }

  // APF 需要 MoveIt 模型来根据当前关节角计算末端位姿。
  auto node_ptr = shared_from_this();
  model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_ptr, "robot_description");
  moveit_model_ = model_loader_->getModel();
  if (!moveit_model_) {
    RCLCPP_FATAL(get_logger(), "Failed to load MoveIt robot model from robot_description");
    throw std::runtime_error("Failed to load MoveIt robot model");
  }

  joint_model_group_ = moveit_model_->getJointModelGroup("arm");
  if (!joint_model_group_) {
    RCLCPP_FATAL(get_logger(), "Failed to get JointModelGroup 'arm'");
    throw std::runtime_error("Failed to get JointModelGroup 'arm'");
  }

  kinematic_state_ = std::make_shared<moveit::core::RobotState>(moveit_model_);
  kinematic_state_->setToDefaultValues();
  apf_initialized_ = true;
}

void ApfPlannerNode::targetTrajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  target_trajectory_ = msg;
}

void ApfPlannerNode::targetStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  target_state_ = msg;
}

void ApfPlannerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (joint_names_.size() != 6) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "APF expects exactly 6 joint names, got %zu", joint_names_.size());
    return;
  }

  Eigen::Matrix<double, 6, 1> positions;
  positions.setZero();

  // 按 joint_names_ 的顺序重排 JointState，保证后续 FK 和控制器使用同一关节顺序。
  for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[joint_idx]);
    if (it == msg->name.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "JointState missing expected joint '%s'", joint_names_[joint_idx].c_str());
      return;
    }

    const size_t msg_idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (msg_idx >= msg->position.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "JointState has no position for '%s'", joint_names_[joint_idx].c_str());
      return;
    }

    positions[static_cast<int>(joint_idx)] = msg->position[msg_idx];
  }

  current_joint_positions_ = positions;
  robot_state_.joint_positions = positions;
  robot_state_.timestamp = now_sec();
  has_joint_state_ = true;

  if (apf_initialized_) {
    updateEndEffectorState();
  }
}

void ApfPlannerNode::planningTimerCallback() {
  if (!apf_initialized_) {
    initialize_APF();
  }

  // 没有关节状态或 FK 失败时不发布新前瞻点，避免 MPC 跟踪过期/错误目标。
  if (!has_joint_state_ || !updateEndEffectorState()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "APF waiting for valid joint state and FK");
    return;
  }

  std::vector<Eigen::Vector3d> lookahead_path;
  if (!buildLookaheadPath(lookahead_path)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "APF waiting for target trajectory or target state");
    return;
  }

  // 一阶低通滤波仍然作用在第一个路径点；同样的平移量应用到整段路径，保持路径形状。
  const Eigen::Vector3d raw_first_point = lookahead_path.front();
  if (has_filtered_lookahead_) {
    filtered_lookahead_ =
        filter_alpha_ * raw_first_point + (1.0 - filter_alpha_) * filtered_lookahead_;
  } else {
    filtered_lookahead_ = raw_first_point;
    has_filtered_lookahead_ = true;
  }

  const Eigen::Vector3d filter_offset = filtered_lookahead_ - raw_first_point;
  for (auto& point : lookahead_path) {
    point = clampWorkspace(point + filter_offset);
  }

  publishLookaheadPath(lookahead_path);
}

bool ApfPlannerNode::updateEndEffectorState() {
  if (!kinematic_state_ || !joint_model_group_) {
    return false;
  }

  kinematic_state_->setJointGroupPositions(joint_model_group_, current_joint_positions_);
  kinematic_state_->update();

  const auto& transform = kinematic_state_->getGlobalLinkTransform(tip_link_name_);
  robot_state_.ee_position = transform.translation();
  robot_state_.ee_orientation = Eigen::Quaterniond(transform.rotation());
  return true;
}

bool ApfPlannerNode::selectTargetPointAtTime(
    double prediction_time,
    Eigen::Vector3d& target_point) const {

  if (target_trajectory_ && !target_trajectory_->poses.empty()) {

    const auto& poses = target_trajectory_->poses;
    const size_t selected_index = targetIndexForPrediction(
        poses, prediction_time, period_ms_ / 1000.0);
    const auto& pose = poses[selected_index].pose.position;

    // 目标只在 XY 平面运动，这里把它提升到固定 z 高度，形成三维任务空间目标。
    target_point = Eigen::Vector3d(pose.x, pose.y, target_plane_z_);
    return true;
  }

  if (target_state_) {
    const auto& pose = target_state_->pose.pose.position;
    const auto& velocity = target_state_->twist.twist.linear;

    // 没有预测轨迹时用 EKF 当前状态和速度外推，同样只在 XY 平面上预测。
    target_point = Eigen::Vector3d(
        pose.x + velocity.x * prediction_time,
        pose.y + velocity.y * prediction_time,
        target_plane_z_);
    return true;
  }

  return false;
}

bool ApfPlannerNode::buildLookaheadPath(std::vector<Eigen::Vector3d>& lookahead_path) const {

  lookahead_path.clear();
  lookahead_path.reserve(static_cast<size_t>(output_path_points_));

  Eigen::Vector3d nominal_point = robot_state_.ee_position;

  for (int step = 0; step < output_path_points_; ++step) 
  {
    Eigen::Vector3d target_point;

    const double prediction_time = target_prediction_time_ + static_cast<double>(step) * output_path_dt_;

    if (!selectTargetPointAtTime(prediction_time, target_point)) {
      return false;
    }

    const Eigen::Vector3d attractive = computeAttractiveForce(nominal_point, target_point);
    const Eigen::Vector3d repulsive = computeRepulsiveForce(nominal_point);

    // 每个路径点都按一次 APF 合力向前滚动，形成给 MPC 使用的短时域参考。
    Eigen::Vector3d force = clampVectorNorm(attractive + repulsive, max_force_norm_);

    if (force.norm() < min_force_norm_) {
      force = target_point - nominal_point;
    }

    if (force.norm() >= min_force_norm_) {
      nominal_point =
          clampWorkspace(nominal_point + lookahead_distance_ * force.normalized());
    }

    lookahead_path.push_back(nominal_point);
  }

  return !lookahead_path.empty();
}

Eigen::Vector3d ApfPlannerNode::computeAttractiveForce(
    const Eigen::Vector3d& from_point,
    const Eigen::Vector3d& target_point) const {
  return attractive_gain_ * (target_point - from_point);
}

Eigen::Vector3d ApfPlannerNode::computeRepulsiveForce(const Eigen::Vector3d& from_point) const {
  Eigen::Vector3d force = Eigen::Vector3d::Zero();

  for (const auto& obstacle : obstacles_) {

    // 障碍物用球体近似；距离超过影响半径后不再产生斥力。
    const Eigen::Vector3d diff = from_point - obstacle.center;
    const double center_distance = diff.norm();
    if (center_distance < 1.0e-9) {
      continue;
    }

    //防穿模保护
    const double clearance = std::max(center_distance - obstacle.radius, obstacle_min_distance_);
    if (clearance >= obstacle_influence_distance_) {
      continue;
    }

    const double magnitude =
        repulsive_gain_ *
        (1.0 / clearance - 1.0 / obstacle_influence_distance_) /
        (clearance * clearance);
    force += magnitude * diff.normalized();
  }

  return force;
}

Eigen::Vector3d ApfPlannerNode::clampVectorNorm(
    const Eigen::Vector3d& value,
    double max_norm) const {
  if (max_norm <= 0.0) {
    return value;
  }

  const double norm = value.norm();
  if (norm <= max_norm || norm < 1.0e-12) {
    return value;
  }

  return value * (max_norm / norm);
}

Eigen::Vector3d ApfPlannerNode::clampWorkspace(const Eigen::Vector3d& point) const {
  Eigen::Vector3d clamped = point;
  // 防止 APF 前瞻点跑出机械臂仿真工作空间。
  for (int i = 0; i < 3; ++i) {
    clamped[i] = std::clamp(clamped[i], workspace_min_[i], workspace_max_[i]);
  }
  return clamped;
}

void ApfPlannerNode::publishLookaheadPath(const std::vector<Eigen::Vector3d>& lookahead_path) {
  if (lookahead_path.empty()) {
    return;
  }

  const auto stamp = now();
  const Eigen::Vector3d& first_point = lookahead_path.front();

  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.stamp = stamp;
  point_msg.header.frame_id = frame_id_;
  point_msg.point.x = first_point.x();
  point_msg.point.y = first_point.y();
  point_msg.point.z = first_point.z();
  lookahead_pub_->publish(point_msg);

  if (!publish_legacy_path_) {
    return;
  }

  // 给 MPC 的短时域 Path，点数和间隔应与 MPC horizon/dt 对齐。
  nav_msgs::msg::Path path_msg;
  path_msg.header = point_msg.header;

  for (size_t i = 0; i < lookahead_path.size(); ++i) {
    const auto& point = lookahead_path[i];
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.header.stamp =
        rclcpp::Time(stamp) + rclcpp::Duration::from_seconds(static_cast<double>(i) * output_path_dt_);
    pose_msg.pose.position.x = point.x();
    pose_msg.pose.position.y = point.y();
    pose_msg.pose.position.z = point.z();
    pose_msg.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose_msg);
  }

  legacy_path_pub_->publish(path_msg);

  visualization_msgs::msg::Marker marker;
  marker.header = path_msg.header;
  marker.ns = "apf_reference";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.015;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  marker.lifetime.nanosec = 200000000;

  marker.points.reserve(lookahead_path.size());
  for (const auto& point : lookahead_path) {
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x();
    marker_point.y = point.y();
    marker_point.z = point.z();
    marker.points.push_back(marker_point);
  }
  marker_pub_->publish(marker);
}

std::vector<ApfPlannerNode::Obstacle> ApfPlannerNode::loadObstaclesFromParameters() const {
  std::vector<Obstacle> obstacles;
  const auto centers = get_parameter("obstacle_centers").as_double_array();
  const auto radii = get_parameter("obstacle_radii").as_double_array();

  const size_t obstacle_count = std::min(centers.size() / 3, radii.size());
  obstacles.reserve(obstacle_count);

  // obstacle_centers 按 x,y,z,x,y,z... 展开，obstacle_radii 给对应半径。
  for (size_t i = 0; i < obstacle_count; ++i) {
    Obstacle obstacle;
    obstacle.center = Eigen::Vector3d(
        centers[i * 3],
        centers[i * 3 + 1],
        centers[i * 3 + 2]);
    obstacle.radius = std::max(0.0, radii[i]);
    obstacles.push_back(obstacle);
  }

  return obstacles;
}

}  // namespace dynamic_assembly

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_assembly::ApfPlannerNode)
