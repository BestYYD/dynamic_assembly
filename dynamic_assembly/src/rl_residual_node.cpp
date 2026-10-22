#include "dynamic_assembly/rl_residual_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cmath>

namespace dynamic_assembly {

namespace {

constexpr size_t kDof = 6;

void appendArray(std_msgs::msg::Float64MultiArray& msg, const std::array<double, kDof>& values) {
  msg.data.insert(msg.data.end(), values.begin(), values.end());
}

void appendZeros(std_msgs::msg::Float64MultiArray& msg, size_t count) {
  msg.data.insert(msg.data.end(), count, 0.0);
}

}  // namespace

RlResidualNode::RlResidualNode(const rclcpp::NodeOptions& options)
    : Node("rl_residual_node", options) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", true);
  }
  if (!get_parameter("use_sim_time").as_bool()) {
    set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  declare_parameter("enabled", enabled_);
  declare_parameter("policy_mode", policy_mode_);
  declare_parameter("publish_corrected_to_controller", publish_corrected_to_controller_);
  declare_parameter("period_ms", period_ms_);
  declare_parameter("max_action_age", max_action_age_);
  declare_parameter("max_residual_velocity", max_residual_velocity_);
  declare_parameter("max_corrected_velocity", max_corrected_velocity_);
  declare_parameter<std::vector<std::string>>("joint_names", joint_names_);
  declare_parameter("joint_state_topic", joint_state_topic_);
  declare_parameter("target_state_topic", target_state_topic_);
  declare_parameter("target_path_topic", target_path_topic_);
  declare_parameter("nominal_topic", nominal_topic_);
  declare_parameter("external_action_topic", external_action_topic_);
  declare_parameter("observation_topic", observation_topic_);
  declare_parameter("residual_action_topic", residual_action_topic_);
  declare_parameter("corrected_topic", corrected_topic_);
  declare_parameter("command_topic", command_topic_);

  enabled_ = get_parameter("enabled").as_bool();
  policy_mode_ = get_parameter("policy_mode").as_string();
  publish_corrected_to_controller_ = get_parameter("publish_corrected_to_controller").as_bool();
  period_ms_ = get_parameter("period_ms").as_double();
  max_action_age_ = get_parameter("max_action_age").as_double();
  max_residual_velocity_ = std::max(0.0, get_parameter("max_residual_velocity").as_double());
  max_corrected_velocity_ = std::max(0.0, get_parameter("max_corrected_velocity").as_double());
  joint_names_ = get_parameter("joint_names").as_string_array();
  joint_state_topic_ = get_parameter("joint_state_topic").as_string();
  target_state_topic_ = get_parameter("target_state_topic").as_string();
  target_path_topic_ = get_parameter("target_path_topic").as_string();
  nominal_topic_ = get_parameter("nominal_topic").as_string();
  external_action_topic_ = get_parameter("external_action_topic").as_string();
  observation_topic_ = get_parameter("observation_topic").as_string();
  residual_action_topic_ = get_parameter("residual_action_topic").as_string();
  corrected_topic_ = get_parameter("corrected_topic").as_string();
  command_topic_ = get_parameter("command_topic").as_string();

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RlResidualNode::jointStateCallback, this, std::placeholders::_1));
  target_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      target_state_topic_, 10,
      std::bind(&RlResidualNode::targetStateCallback, this, std::placeholders::_1));
  target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      target_path_topic_, 10,
      std::bind(&RlResidualNode::targetPathCallback, this, std::placeholders::_1));
  nominal_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      nominal_topic_, 10,
      std::bind(&RlResidualNode::nominalCommandCallback, this, std::placeholders::_1));
  external_action_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      external_action_topic_, 10,
      std::bind(&RlResidualNode::externalActionCallback, this, std::placeholders::_1));

  observation_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(observation_topic_, 10);
  residual_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(residual_action_topic_, 10);
  corrected_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(corrected_topic_, 10);
  if (publish_corrected_to_controller_) {
    controller_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(command_topic_, 10);
  }

  timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(std::max(1.0, period_ms_))),
      std::bind(&RlResidualNode::timerCallback, this));

  RCLCPP_INFO(
      get_logger(),
      "RL residual node initialized: enabled=%s, policy_mode=%s, controller_output=%s",
      enabled_ ? "true" : "false",
      policy_mode_.c_str(),
      publish_corrected_to_controller_ ? "true" : "false");
}

void RlResidualNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (joint_names_.size() != kDof) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "RL node expects 6 joint names, got %zu", joint_names_.size());
    return;
  }

  std::array<double, kDof> positions{};
  std::array<double, kDof> velocities{};
  for (size_t joint_idx = 0; joint_idx < kDof; ++joint_idx) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[joint_idx]);
    if (it == msg->name.end()) {
      return;
    }
    const size_t msg_idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (msg_idx >= msg->position.size()) {
      return;
    }
    positions[joint_idx] = msg->position[msg_idx];
    if (msg_idx < msg->velocity.size()) {
      velocities[joint_idx] = msg->velocity[msg_idx];
    }
  }

  joint_positions_ = positions;
  joint_velocities_ = velocities;
  last_joint_state_time_ = now().seconds();
  has_joint_state_ = true;
}

void RlResidualNode::targetStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  target_state_ = msg;
  last_target_state_time_ = now().seconds();
  has_target_state_ = true;
}

void RlResidualNode::targetPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  target_path_ = msg;
  last_target_path_time_ = now().seconds();
  has_target_path_ = true;
}

void RlResidualNode::nominalCommandCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  nominal_command_ = msg;
  last_nominal_time_ = now().seconds();
  has_nominal_command_ = true;
}

void RlResidualNode::externalActionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  for (size_t i = 0; i < kDof; ++i) {
    external_action_[i] = (i < msg->data.size()) ? msg->data[i] : 0.0;
  }
  last_external_action_time_ = now().seconds();
  has_external_action_ = true;
}

void RlResidualNode::timerCallback() {
  const double now_time = now().seconds();
  observation_pub_->publish(buildObservation());

  const auto residual = selectResidualAction(now_time);
  residual_pub_->publish(makeResidualMessage(residual));

  if (!has_nominal_command_ || !nominal_command_) {
    return;
  }

  const auto corrected = applyResidualToNominal(*nominal_command_, residual);
  corrected_pub_->publish(corrected);

  // 默认不发布到真实控制器；只有显式打开参数时才让 RL 接管输出。
  if (publish_corrected_to_controller_ && controller_pub_) {
    controller_pub_->publish(corrected);
  }
}

std_msgs::msg::Float64MultiArray RlResidualNode::buildObservation() const {
  std_msgs::msg::Float64MultiArray obs;
  obs.layout.dim.resize(1);
  obs.layout.dim[0].label = "rl_observation";

  // 观测向量布局：
  // q(6), dq(6), target_xyz(3), target_vxyz(3), path0_xyz(3),
  // nominal_q0(6), nominal_dq0(6), valid_flags(5)
  appendArray(obs, has_joint_state_ ? joint_positions_ : std::array<double, kDof>{});
  appendArray(obs, has_joint_state_ ? joint_velocities_ : std::array<double, kDof>{});

  if (has_target_state_ && target_state_) {
    const auto& p = target_state_->pose.pose.position;
    obs.data.insert(obs.data.end(), {p.x, p.y, p.z});
    const auto& v = target_state_->twist.twist.linear;
    obs.data.insert(obs.data.end(), {v.x, v.y, v.z});
  } else {
    appendZeros(obs, 6);
  }

  if (has_target_path_ && target_path_ && !target_path_->poses.empty()) {
    const auto& p = target_path_->poses.front().pose.position;
    obs.data.insert(obs.data.end(), {p.x, p.y, p.z});
  } else {
    appendZeros(obs, 3);
  }

  if (has_nominal_command_ && nominal_command_ && !nominal_command_->points.empty()) {
    const auto& p0 = nominal_command_->points.front();
    for (size_t i = 0; i < kDof; ++i) {
      obs.data.push_back(i < p0.positions.size() ? p0.positions[i] : 0.0);
    }
    for (size_t i = 0; i < kDof; ++i) {
      obs.data.push_back(i < p0.velocities.size() ? p0.velocities[i] : 0.0);
    }
  } else {
    appendZeros(obs, 12);
  }

  obs.data.insert(obs.data.end(), {
      has_joint_state_ ? 1.0 : 0.0,
      has_target_state_ ? 1.0 : 0.0,
      has_target_path_ ? 1.0 : 0.0,
      has_nominal_command_ ? 1.0 : 0.0,
      enabled_ ? 1.0 : 0.0});

  obs.layout.dim[0].size = obs.data.size();
  obs.layout.dim[0].stride = obs.data.size();
  return obs;
}

std::array<double, 6> RlResidualNode::selectResidualAction(double now_time) const {
  std::array<double, kDof> residual{};

  if (!enabled_) {
    return residual;
  }

  if (policy_mode_ == "topic") {
    const bool action_fresh =
        has_external_action_ && messageAge(now_time, last_external_action_time_) <= max_action_age_;
    if (!action_fresh) {
      return residual;
    }

    // topic 模式下外部动作按 [-1, 1] 归一化输入，再映射为关节速度残差。
    for (size_t i = 0; i < kDof; ++i) {
      residual[i] = clampResidual(external_action_[i] * max_residual_velocity_);
    }
    return residual;
  }

  // zero 模式是安全占位策略；后续 ONNX/LibTorch 推理可以从这里接入。
  return residual;
}

trajectory_msgs::msg::JointTrajectory RlResidualNode::applyResidualToNominal(
    const trajectory_msgs::msg::JointTrajectory& nominal,
    const std::array<double, 6>& residual) const {
  auto corrected = nominal;
  corrected.header.stamp.sec = 0;
  corrected.header.stamp.nanosec = 0;

  for (auto& point : corrected.points) {
    if (point.velocities.size() < kDof) {
      point.velocities.resize(kDof, 0.0);
    }
    for (size_t i = 0; i < kDof; ++i) {
      point.velocities[i] = clampCorrectedVelocity(point.velocities[i] + residual[i]);
    }

    // 仅在已有 positions 时做一致性修正，避免构造不完整轨迹。
    if (point.positions.size() >= kDof) {
      const double t = durationToSeconds(point.time_from_start);
      for (size_t i = 0; i < kDof; ++i) {
        point.positions[i] += residual[i] * t;
      }
    }
  }

  return corrected;
}

std_msgs::msg::Float64MultiArray RlResidualNode::makeResidualMessage(
    const std::array<double, 6>& residual) const {
  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "joint_velocity_residual";
  msg.layout.dim[0].size = kDof;
  msg.layout.dim[0].stride = kDof;
  msg.data.assign(residual.begin(), residual.end());
  return msg;
}

double RlResidualNode::clampResidual(double value) const {
  return std::clamp(value, -max_residual_velocity_, max_residual_velocity_);
}

double RlResidualNode::clampCorrectedVelocity(double value) const {
  if (max_corrected_velocity_ <= 0.0) {
    return value;
  }
  return std::clamp(value, -max_corrected_velocity_, max_corrected_velocity_);
}

double RlResidualNode::messageAge(double now_time, double stamp_time) const {
  return std::max(0.0, now_time - stamp_time);
}

double RlResidualNode::stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double RlResidualNode::durationToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

}  // namespace dynamic_assembly

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_assembly::RlResidualNode)
