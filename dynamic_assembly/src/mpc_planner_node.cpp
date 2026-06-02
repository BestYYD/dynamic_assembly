#include "dynamic_assembly/mpc_planner_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <osqp.h>

namespace dynamic_assembly {

namespace {

constexpr int kDof = 6;
constexpr double kSparseTolerance = 1.0e-12;

// QP 变量按时间步展开：第 step 步第 joint 个关节速度的位置。
int controlIndex(int step, int joint) {
  return step * kDof + joint;
}

// ROS 时间戳统一转成秒，方便按 MPC 预测步长匹配目标路径。
double stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromQuaternion(const Eigen::Quaterniond& quaternion) {
  const Eigen::Quaterniond normalized = quaternion.normalized();
  const double siny_cosp =
      2.0 * (normalized.w() * normalized.z() + normalized.x() * normalized.y());
  const double cosy_cosp =
      1.0 - 2.0 * (normalized.y() * normalized.y() + normalized.z() * normalized.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

struct SparseMatrixData {
  std::vector<c_int> col_ptrs;
  std::vector<c_int> row_indices;
  std::vector<c_float> values;
};

// OSQP 使用 CSC 稀疏矩阵；P 矩阵只需要上传上三角部分。
SparseMatrixData denseToCsc(const Eigen::MatrixXd& dense, bool upper_triangular_only) {
  SparseMatrixData sparse;
  sparse.col_ptrs.reserve(static_cast<size_t>(dense.cols()) + 1);
  sparse.col_ptrs.push_back(0);

  for (int col = 0; col < dense.cols(); ++col) {
    const int last_row = static_cast<int>(dense.rows()) - 1;
    const int row_end = upper_triangular_only ? std::min(col, last_row) : last_row;
    for (int row = 0; row <= row_end; ++row) {
      const double value = dense(row, col);
      if (std::abs(value) > kSparseTolerance) {
        sparse.row_indices.push_back(static_cast<c_int>(row));
        sparse.values.push_back(static_cast<c_float>(value));
      }
    }
    sparse.col_ptrs.push_back(static_cast<c_int>(sparse.values.size()));
  }

  return sparse;
}

// 把一项 weight * ||coefficients^T u - target||^2 加入二次规划目标函数。
void addLeastSquaresTerm(
    Eigen::MatrixXd& hessian,
    Eigen::VectorXd& gradient,
    const Eigen::VectorXd& coefficients,
    double target,
    double weight) {
  if (weight <= 0.0) {
    return;
  }

  hessian.noalias() += 2.0 * weight * (coefficients * coefficients.transpose());
  gradient.noalias() -= 2.0 * weight * target * coefficients;
}

}  // namespace

MpcPlannerNode::MpcPlannerNode(const rclcpp::NodeOptions& options)
    : Node("mpc_planner_node", options) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", true);
  }
  if (!get_parameter("use_sim_time").as_bool()) {
    set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  // 参数保留在 ROS 参数服务器里，便于在不重新编译的情况下调频率、权重和 OSQP 精度。
  declare_parameter("horizon", 20);
  declare_parameter("dt", 0.02);
  declare_parameter("timeout_ms", 18.0);
  declare_parameter("tracking_weight", 1.0);
  declare_parameter("control_weight", 0.01);
  declare_parameter("max_tracking_error", 0.1);
  declare_parameter("max_data_age", 0.15);
  declare_parameter("command_topic", command_topic_);
  declare_parameter("nominal_topic", nominal_topic_);
  declare_parameter("goal_path_topic", goal_path_topic_);
  declare_parameter("joint_state_topic", joint_state_topic_);
  declare_parameter("approach_height", approach_height_);
  declare_parameter("cartesian_z_weight", cartesian_z_weight_);
  declare_parameter("linearization_damping", linearization_damping_);
  declare_parameter("qp_regularization", qp_regularization_);
  declare_parameter("max_joint_velocity", max_joint_velocity_);
  declare_parameter("terminal_weight", terminal_weight_);
  declare_parameter("control_delta_weight", control_delta_weight_);
  declare_parameter("track_planar_yaw", track_planar_yaw_);
  declare_parameter("z_axis_alignment_weight", z_axis_alignment_weight_);
  declare_parameter("yaw_tracking_weight", yaw_tracking_weight_);
  declare_parameter("yaw_terminal_weight", yaw_terminal_weight_);
  declare_parameter("osqp_max_iterations", osqp_max_iterations_);
  declare_parameter("osqp_eps_abs", osqp_eps_abs_);
  declare_parameter("osqp_eps_rel", osqp_eps_rel_);

  config_.horizon = get_parameter("horizon").as_int();
  config_.dt = get_parameter("dt").as_double();
  config_.timeout_ms = get_parameter("timeout_ms").as_double();
  config_.tracking_weight = get_parameter("tracking_weight").as_double();
  config_.control_weight = get_parameter("control_weight").as_double();
  max_data_age_ = get_parameter("max_data_age").as_double();
  command_topic_ = get_parameter("command_topic").as_string();
  nominal_topic_ = get_parameter("nominal_topic").as_string();
  goal_path_topic_ = get_parameter("goal_path_topic").as_string();
  joint_state_topic_ = get_parameter("joint_state_topic").as_string();
  approach_height_ = get_parameter("approach_height").as_double();
  cartesian_z_weight_ = get_parameter("cartesian_z_weight").as_double();
  linearization_damping_ = get_parameter("linearization_damping").as_double();
  qp_regularization_ = get_parameter("qp_regularization").as_double();
  max_joint_velocity_ = get_parameter("max_joint_velocity").as_double();
  terminal_weight_ = get_parameter("terminal_weight").as_double();
  control_delta_weight_ = get_parameter("control_delta_weight").as_double();
  track_planar_yaw_ = get_parameter("track_planar_yaw").as_bool();
  z_axis_alignment_weight_ = get_parameter("z_axis_alignment_weight").as_double();
  yaw_tracking_weight_ = get_parameter("yaw_tracking_weight").as_double();
  yaw_terminal_weight_ = get_parameter("yaw_terminal_weight").as_double();
  osqp_max_iterations_ = get_parameter("osqp_max_iterations").as_int();
  osqp_eps_abs_ = get_parameter("osqp_eps_abs").as_double();
  osqp_eps_rel_ = get_parameter("osqp_eps_rel").as_double();

  current_joint_positions_.setZero();
  current_joint_velocities_.setZero();

  goal_sub_ = create_subscription<nav_msgs::msg::Path>(
      goal_path_topic_, 10,
      std::bind(&MpcPlannerNode::goalPathCallback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MpcPlannerNode::jointStateCallback, this, std::placeholders::_1));

  traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      command_topic_, 10);
  nominal_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      nominal_topic_, 10);

  timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(config_.dt * 1000)),
      std::bind(&MpcPlannerNode::controlTimerCallback, this));

  RCLCPP_INFO(get_logger(), "MPC planner initialized, freq: %.0fHz", 
              1.0 / config_.dt);
  RCLCPP_INFO(get_logger(), "MPC goal path: %s, command topic: %s, nominal topic: %s",
              goal_path_topic_.c_str(), command_topic_.c_str(), nominal_topic_.c_str());
}

void MpcPlannerNode::goalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  // 这里接收 APF 包装后的前瞻点 Path，也兼容未来多点参考路径。
  goal_path_ = msg;
  last_goal_time_ = now_sec();
}

void MpcPlannerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  Eigen::Matrix<double, 6, 1> positions;
  Eigen::Matrix<double, 6, 1> velocities;
  positions.setZero();
  velocities.setZero();

  // 按固定关节名提取状态，避免 JointState 消息顺序变化导致控制量错位。
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

    positions[joint_idx] = msg->position[msg_idx];
    if (msg_idx < msg->velocity.size()) {
      velocities[joint_idx] = msg->velocity[msg_idx];
    }
  }

  current_joint_positions_ = positions;
  current_joint_velocities_ = velocities;
  robot_state_.joint_positions = positions;
  robot_state_.joint_velocities = velocities;
  robot_state_.timestamp = now_sec();
  last_joint_state_time_ = robot_state_.timestamp;
  has_joint_state_ = true;

  if (kinematic_state_ && joint_model_group_) {
    // 同步 MoveIt 状态，供后续 FK 和雅可比计算使用。
    kinematic_state_->setJointGroupPositions(joint_model_group_, current_joint_positions_);
    kinematic_state_->update();

    const auto& ee_transform = kinematic_state_->getGlobalLinkTransform(tip_link_name_);
    robot_state_.ee_position = ee_transform.translation();
    robot_state_.ee_orientation = Eigen::Quaterniond(ee_transform.rotation());
  }
}

void MpcPlannerNode::initialize_MPC(){
  // 加载 MoveIt 模型后，MPC 可以实时计算 FK 和雅可比，建立笛卡尔 LTV 模型。
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

  joint_lower_limits_.setConstant(-std::numeric_limits<double>::infinity());
  joint_upper_limits_.setConstant(std::numeric_limits<double>::infinity());
  const auto& joint_models = joint_model_group_->getActiveJointModels();
  // 从 URDF/MoveIt 模型读取关节上下限，作为 MPC 预测过程中的硬约束。
  for (size_t i = 0; i < joint_models.size() && i < kDof; ++i) {
    const auto& bounds = joint_models[i]->getVariableBounds(joint_models[i]->getName());
    if (std::isfinite(bounds.min_position_) && std::isfinite(bounds.max_position_) &&
        bounds.min_position_ < bounds.max_position_) {
      joint_lower_limits_[static_cast<int>(i)] = bounds.min_position_;
      joint_upper_limits_[static_cast<int>(i)] = bounds.max_position_;
    }
  }
}

void MpcPlannerNode::controlTimerCallback() {

  if (!mpc_initialized_) {
    initialize_MPC();
    mpc_initialized_ = true;
  }
  double current_time = now_sec();

  // 目标或关节状态过期时不求解 MPC，直接进入保持策略。
  if (!goal_path_ || (current_time - last_goal_time_) > max_data_age_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                         "Goal path data too old or missing, executing fallback");
    executeFallback();
    return;
  }

  if (!has_joint_state_ || (current_time - last_joint_state_time_) > max_data_age_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Joint state data too old or missing, holding current command");
    executeFallback();
    return;
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  auto start = std::chrono::steady_clock::now();

  // 每个控制周期求解一次有限时域 QP，超时则不用该次解，保证控制周期稳定。
  bool success = solveMpc(trajectory);
  auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

  if (!success || elapsed > config_.timeout_ms) {
    RCLCPP_WARN(get_logger(), "MPC solve failed or timeout (%.1fms), fallback", elapsed);
    executeFallback();
    return;
  }

  // A zero stamp makes joint_trajectory_controller execute the rolling MPC
  // command immediately, independent of wall-time vs. sim-time drift.
  trajectory.header.stamp.sec = 0;
  trajectory.header.stamp.nanosec = 0;
  nominal_traj_pub_->publish(trajectory);
  traj_pub_->publish(trajectory);
  last_trajectory_ = trajectory;
  has_last_trajectory_ = true;
}

bool MpcPlannerNode::solveMpc(trajectory_msgs::msg::JointTrajectory& output) {
  if (!goal_path_ || goal_path_->poses.empty() || !kinematic_state_ || !joint_model_group_) {
    return false;
  }

  output.joint_names = joint_names_;

  Eigen::Matrix<double, 6, 6> current_jacobian;
  Eigen::Vector3d current_ee_position;
  Eigen::Vector3d current_tool_z_axis;
  double current_ee_yaw = 0.0;
  if (!computePlanarPoseJacobian(
          current_joint_positions_, current_jacobian, &current_ee_position, &current_tool_z_axis, &current_ee_yaw)) {
    return false;
  }

  if (track_planar_yaw_ && !has_locked_yaw_reference_) {
    locked_yaw_reference_ = current_ee_yaw;
    has_locked_yaw_reference_ = true;
  } else if (!track_planar_yaw_) {
    has_locked_yaw_reference_ = false;
  }

  // 第一步：使用 APF 给出路径点；开启 yaw 追踪时锁住当前世界系 yaw。
  std::vector<PlanarPoseReference> references;
  if (!buildReferencePlanarPoseTrajectory(references, current_ee_yaw)) {
    return false;
  }

  // 第二步：沿名义关节轨迹计算每个预测步的 J(q_k)，形成 LTV 运动学模型。
  std::vector<Eigen::Matrix<double, 6, 6>> jacobians;
  if (!buildLinearizedKinematicModel(references, jacobians)) {
    return false;
  }
  jacobians.front() = current_jacobian;

  Eigen::Matrix<double, 6, 1> current_planar_pose;
  current_planar_pose << current_ee_position.x(), current_ee_position.y(), current_ee_position.z(),
      current_tool_z_axis.x(), current_tool_z_axis.y(), current_ee_yaw;

  // 第三步：求解关节速度序列，使末端 XYZ、工具 Z 轴和 yaw 贴近参考。
  std::vector<Eigen::Matrix<double, kDof, 1>> optimal_velocities;
  if (!solveCartesianLtvMpc(
          current_planar_pose,
          current_joint_positions_,
          current_joint_velocities_,
          references,
          jacobians,
          optimal_velocities)) {
    return false;
  }

  Eigen::Matrix<double, kDof, 1> q_curr = current_joint_positions_;
  
  // 第四步：把最优速度积分成 JointTrajectory，供 ros2_control 执行。
  for (size_t i = 0; i < optimal_velocities.size(); ++i) {
    const Eigen::Matrix<double, kDof, 1> q_dot = optimal_velocities[i];
    q_curr += config_.dt * q_dot;
    clampToJointLimits(q_curr);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    fillJointTrajectoryPoint(q_curr, q_dot, (static_cast<double>(i) + 1.0) * config_.dt, point);
    output.points.push_back(point);
  }

  if (!output.points.empty()) {
    auto& terminal_point = output.points.back();
    terminal_point.velocities.assign(joint_names_.size(), 0.0);
  }

  return !output.points.empty();
}

bool MpcPlannerNode::buildReferencePlanarPoseTrajectory(
    std::vector<PlanarPoseReference>& references,
    double current_yaw) const {

  if (!goal_path_ || goal_path_->poses.empty()) {
    return false;
  }

  references.clear();
  references.reserve(static_cast<size_t>(config_.horizon));
  const double reference_yaw =
      (track_planar_yaw_ && has_locked_yaw_reference_) ? locked_yaw_reference_ : current_yaw;

  for (int step = 0; step < config_.horizon; ++step) {

    //算出第 step 步对应未来的哪个时间，然后在路径数组里找到那个特定时间的点返回
    const size_t pose_index = targetPoseIndexForStep(step);
    const auto& path_pose = goal_path_->poses[pose_index].pose;

    PlanarPoseReference reference;
    // 平面跟踪时 APF 只提供 XY 参考，末端高度由 MPC 的 approach_height 固定。
    reference.position = Eigen::Vector3d(path_pose.position.x, path_pose.position.y, approach_height_);
    reference.yaw = reference_yaw;
    references.push_back(reference);
  }

  return !references.empty();
}

bool MpcPlannerNode::buildLinearizedKinematicModel(
    const std::vector<PlanarPoseReference>& references,
    std::vector<Eigen::Matrix<double, 6, 6>>& jacobians) {
  if (references.empty() || !kinematic_state_ || !joint_model_group_) {
    return false;
  }

  jacobians.clear();
  jacobians.reserve(references.size());

  Eigen::Matrix<double, kDof, 1> q_nominal = current_joint_positions_;
  for (const auto& reference : references) {
    Eigen::Matrix<double, 6, 6> jacobian;
    Eigen::Vector3d ee_position;
    Eigen::Vector3d tool_z_axis;
    double ee_yaw = 0.0;
    if (!computePlanarPoseJacobian(q_nominal, jacobian, &ee_position, &tool_z_axis, &ee_yaw)) {
      return false;
    }

    jacobians.push_back(jacobian);

    // 用阻尼最小二乘生成一条名义轨迹，下一步在该名义点重新线性化。
    Eigen::Matrix<double, 6, 1> planar_pose_error;
    planar_pose_error << reference.position - ee_position,
        track_planar_yaw_ ? -tool_z_axis.x() : 0.0,
        track_planar_yaw_ ? -tool_z_axis.y() : 0.0,
        track_planar_yaw_ ? normalizeAngle(reference.yaw - ee_yaw) : 0.0;
    const Eigen::Matrix<double, kDof, 1> q_dot_nominal =
        dampedLeastSquaresVelocity(jacobian, planar_pose_error);
    q_nominal += config_.dt * q_dot_nominal;
    clampToJointLimits(q_nominal);
  }

  // 恢复到当前真实关节状态，避免名义线性化过程污染节点里的 MoveIt 状态。
  kinematic_state_->setJointGroupPositions(joint_model_group_, current_joint_positions_);
  kinematic_state_->update();
  return jacobians.size() == references.size();
}

bool MpcPlannerNode::computePlanarPoseJacobian(
    const Eigen::Matrix<double, 6, 1>& q,
    Eigen::Matrix<double, 6, 6>& jacobian,
    Eigen::Vector3d* ee_position,
    Eigen::Vector3d* tool_z_axis,
    double* ee_yaw) {
  if (!kinematic_state_ || !joint_model_group_ || !moveit_model_) {
    return false;
  }

  const moveit::core::LinkModel* tip_link = moveit_model_->getLinkModel(tip_link_name_);
  if (!tip_link) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Tip link '%s' not found while computing Jacobian",
                         tip_link_name_.c_str());
    return false;
  }

  kinematic_state_->setJointGroupPositions(joint_model_group_, q);
  kinematic_state_->update();

  Eigen::MatrixXd full_jacobian;
  if (!kinematic_state_->getJacobian(
          joint_model_group_, tip_link, Eigen::Vector3d::Zero(), full_jacobian)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed to compute end-effector Jacobian");
    return false;
  }

  if (full_jacobian.rows() < 6 || full_jacobian.cols() < kDof) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Unexpected Jacobian size: %ld x %ld",
                         full_jacobian.rows(), full_jacobian.cols());
    return false;
  }

  const auto& ee_transform = kinematic_state_->getGlobalLinkTransform(tip_link_name_);
  const Eigen::Matrix<double, 3, 6> angular_jacobian = full_jacobian.block<3, kDof>(3, 0);
  const Eigen::Vector3d tool_z = ee_transform.rotation().col(2).normalized();
  Eigen::Matrix3d tool_z_cross;
  tool_z_cross << 0.0, -tool_z.z(), tool_z.y(),
      tool_z.z(), 0.0, -tool_z.x(),
      -tool_z.y(), tool_z.x(), 0.0;

  jacobian.topRows<3>() = full_jacobian.topLeftCorner<3, kDof>();
  jacobian.middleRows<2>(3) = (-tool_z_cross * angular_jacobian).topRows<2>();
  jacobian.row(5) = angular_jacobian.row(2);
  if (ee_position) {
    *ee_position = ee_transform.translation();
  }
  if (tool_z_axis) {
    *tool_z_axis = tool_z;
  }
  if (ee_yaw) {
    *ee_yaw = yawFromQuaternion(Eigen::Quaterniond(ee_transform.rotation()));
  }
  return true;
}

//求关节速度
Eigen::Matrix<double, 6, 1> MpcPlannerNode::dampedLeastSquaresVelocity(
    const Eigen::Matrix<double, 6, 6>& jacobian,
    const Eigen::Matrix<double, 6, 1>& planar_pose_error) const {

  const double dt = std::max(1.0e-3, config_.dt);
  const double damping = std::max(1.0e-6, linearization_damping_);
  Eigen::Matrix<double, 6, 1> desired_planar_pose_velocity = planar_pose_error / dt;
  if (!track_planar_yaw_) {
    desired_planar_pose_velocity.tail<3>().setZero();
  }

  Eigen::Matrix<double, 6, 6> task_matrix = jacobian * jacobian.transpose();
  task_matrix.diagonal().array() += damping * damping;

  Eigen::Matrix<double, kDof, 1> q_dot =
      jacobian.transpose() * task_matrix.ldlt().solve(desired_planar_pose_velocity);
  return clampJointVelocity(q_dot);
}

std::size_t MpcPlannerNode::targetPoseIndexForStep(int step) const {
  if (!goal_path_ || goal_path_->poses.empty()) {
    return 0;
  }

  const auto& poses = goal_path_->poses;
  if (poses.size() == 1) {
    return 0;
  }

  const double first_stamp = stampToSeconds(poses.front().header.stamp);
  const double last_stamp = stampToSeconds(poses.back().header.stamp);
  if (last_stamp > first_stamp) {
    // 有有效时间戳时，按 MPC 第 step 步对应的 APF 路径时间取参考点。
    const double query_time = first_stamp + static_cast<double>(step) * config_.dt;
    for (size_t i = 0; i < poses.size(); ++i) {
      if (stampToSeconds(poses[i].header.stamp) >= query_time) {
        return i;
      }
    }
    return poses.size() - 1;
  }

  // 没有时间戳时，退化为按数组下标取点。
  return std::min(static_cast<size_t>(step), poses.size() - 1);
}

bool MpcPlannerNode::solveCartesianLtvMpc(
    const Eigen::Matrix<double, 6, 1>& x0,
    const Eigen::Matrix<double, 6, 1>& q0,
    const Eigen::Matrix<double, 6, 1>& u_previous,
    const std::vector<PlanarPoseReference>& references,
    const std::vector<Eigen::Matrix<double, 6, 6>>& jacobians,
    std::vector<Eigen::Matrix<double, 6, 1>>& optimal_velocities) {

  const int horizon = static_cast<int>(references.size());
  if (horizon <= 0 || jacobians.size() != references.size()) {
    return false;
  }

  const int num_variables = horizon * kDof;
  const int num_constraints = 2 * num_variables;
  const double dt = config_.dt;

  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(num_variables, num_variables);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_variables);

  const double tracking_weight = std::max(0.0, config_.tracking_weight);
  const double terminal_weight = std::max(0.0, terminal_weight_);
  const double z_weight = std::max(0.0, cartesian_z_weight_);
  const double z_axis_weight = track_planar_yaw_ ? std::max(0.0, z_axis_alignment_weight_) : 0.0;
  const double yaw_weight = track_planar_yaw_ ? std::max(0.0, yaw_tracking_weight_) : 0.0;
  const double yaw_terminal_weight = track_planar_yaw_ ? std::max(0.0, yaw_terminal_weight_) : 0.0;
  const double control_weight = std::max(0.0, config_.control_weight);
  const double delta_weight = std::max(0.0, control_delta_weight_);
  const Eigen::Matrix<double, 6, 1> axis_weights(
      tracking_weight, tracking_weight, z_weight, z_axis_weight, z_axis_weight, yaw_weight);

  // QP 变量：[u_0, u_1, ..., u_{N-1}]，每个 u_k 是 6 维关节速度。
  // LTV 预测模型：x_k = x0 + dt * Σ J_i(q_nom_i) * u_i，其中 x 是 XYZ、工具 Z 轴误差和 yaw。
  for (int step = 0; step < horizon; ++step) {
    const double terminal_bonus = (step == horizon - 1) ? terminal_weight : 0.0;
    for (int axis = 0; axis < 6; ++axis) {
      Eigen::VectorXd coefficients = Eigen::VectorXd::Zero(num_variables);
      for (int control_step = 0; control_step <= step; ++control_step) {
        for (int joint = 0; joint < kDof; ++joint) {
          coefficients[controlIndex(control_step, joint)] +=
              dt * jacobians[static_cast<size_t>(control_step)](axis, joint);
        }
      }

      double target = 0.0;
      if (axis < 3) {
        target = references[static_cast<size_t>(step)].position[axis] - x0[axis];
      } else if (axis < 5) {
        target = -x0[axis];
      } else {
        target = normalizeAngle(references[static_cast<size_t>(step)].yaw - x0[5]);
      }
      const double axis_terminal_bonus = (axis == 5) ? yaw_terminal_weight : (axis < 3 ? terminal_bonus : 0.0);
      addLeastSquaresTerm(
          hessian, gradient, coefficients, target, axis_weights[axis] + axis_terminal_bonus);
    }
  }

  // 控制代价抑制速度过大；速度变化代价抑制相邻控制量突变，让轨迹更平滑。
  for (int step = 0; step < horizon; ++step) {
    for (int joint = 0; joint < kDof; ++joint) {
      const int idx = controlIndex(step, joint);
      hessian(idx, idx) += 2.0 * control_weight;

      Eigen::VectorXd coefficients = Eigen::VectorXd::Zero(num_variables);
      coefficients[idx] = 1.0;
      if (step == 0) {
        addLeastSquaresTerm(hessian, gradient, coefficients, u_previous[joint], delta_weight);
      } else {
        coefficients[controlIndex(step - 1, joint)] = -1.0;
        addLeastSquaresTerm(hessian, gradient, coefficients, 0.0, delta_weight);
      }
    }
  }

  // 正则化保证 Hessian 正定/半正定更稳定，减少 OSQP 数值问题。
  hessian.diagonal().array() += std::max(1.0e-9, qp_regularization_);

  Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(num_constraints, num_variables);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(num_constraints);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(num_constraints);

  int row = 0;
  // 约束 1：每个预测步的关节速度都不能超过 max_joint_velocity_。
  for (int variable = 0; variable < num_variables; ++variable) {
    constraints(row, variable) = 1.0;
    lower[row] = -max_joint_velocity_;
    upper[row] = max_joint_velocity_;
    ++row;
  }

  // 约束 2：积分后的预测关节位置不能越过 MoveIt/URDF 中定义的关节限位。
  for (int step = 0; step < horizon; ++step) {
    for (int joint = 0; joint < kDof; ++joint) {
      for (int control_step = 0; control_step <= step; ++control_step) {
        constraints(row, controlIndex(control_step, joint)) = dt;
      }
      lower[row] = std::isfinite(joint_lower_limits_[joint])
          ? joint_lower_limits_[joint] - q0[joint]
          : -static_cast<double>(OSQP_INFTY);
      upper[row] = std::isfinite(joint_upper_limits_[joint])
          ? joint_upper_limits_[joint] - q0[joint]
          : static_cast<double>(OSQP_INFTY);
      ++row;
    }
  }

  // 转成 OSQP 需要的稀疏 CSC 格式，避免把稠密矩阵直接传给求解器。
  SparseMatrixData p_sparse = denseToCsc(hessian, true);
  SparseMatrixData a_sparse = denseToCsc(constraints, false);
  csc* p_csc = csc_matrix(
      static_cast<c_int>(num_variables),
      static_cast<c_int>(num_variables),
      static_cast<c_int>(p_sparse.values.size()),
      p_sparse.values.data(),
      p_sparse.row_indices.data(),
      p_sparse.col_ptrs.data());
  csc* a_csc = csc_matrix(
      static_cast<c_int>(num_constraints),
      static_cast<c_int>(num_variables),
      static_cast<c_int>(a_sparse.values.size()),
      a_sparse.values.data(),
      a_sparse.row_indices.data(),
      a_sparse.col_ptrs.data());

  if (!p_csc || !a_csc) {
    if (p_csc) {
      c_free(p_csc);
    }
    if (a_csc) {
      c_free(a_csc);
    }
    return false;
  }

  std::vector<c_float> q_data(static_cast<size_t>(num_variables));
  std::vector<c_float> lower_data(static_cast<size_t>(num_constraints));
  std::vector<c_float> upper_data(static_cast<size_t>(num_constraints));
  for (int i = 0; i < num_variables; ++i) {
    q_data[static_cast<size_t>(i)] = static_cast<c_float>(gradient[i]);
  }
  for (int i = 0; i < num_constraints; ++i) {
    lower_data[static_cast<size_t>(i)] = static_cast<c_float>(lower[i]);
    upper_data[static_cast<size_t>(i)] = static_cast<c_float>(upper[i]);
  }

  OSQPData data{};
  data.n = static_cast<c_int>(num_variables);
  data.m = static_cast<c_int>(num_constraints);
  data.P = p_csc;
  data.A = a_csc;
  data.q = q_data.data();
  data.l = lower_data.data();
  data.u = upper_data.data();

  // OSQP 参数里设置迭代次数、收敛精度和时间限制，防止求解阻塞控制周期。
  OSQPSettings settings{};
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 0;
  settings.warm_start = 1;
  settings.max_iter = static_cast<c_int>(std::max(1, osqp_max_iterations_));
  settings.eps_abs = static_cast<c_float>(std::max(1.0e-6, osqp_eps_abs_));
  settings.eps_rel = static_cast<c_float>(std::max(1.0e-6, osqp_eps_rel_));
  settings.time_limit = static_cast<c_float>(std::max(0.001, config_.timeout_ms / 1000.0));

  OSQPWorkspace* work = nullptr;
  const c_int setup_status = osqp_setup(&work, &data, &settings);
  bool success = false;

  //求解器开始求解
  if (setup_status == 0 && work) {
    const c_int solve_status = osqp_solve(work);
    const c_int status = work->info ? work->info->status_val : 0;
    success = (solve_status == 0) && (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE);

    if (success && work->solution && work->solution->x) {
      optimal_velocities.clear();
      optimal_velocities.reserve(static_cast<size_t>(horizon));

      // 只取 OSQP 给出的最优速度序列，再做一次限幅作为安全兜底。
      for (int step = 0; step < horizon; ++step) {
        Eigen::VectorXd raw_velocity(kDof);
        for (int joint = 0; joint < kDof; ++joint) {
          raw_velocity[joint] = static_cast<double>(work->solution->x[controlIndex(step, joint)]);
        }
        optimal_velocities.push_back(clampJointVelocity(raw_velocity));
      }
    } else {
      RCLCPP_WARN(get_logger(), "OSQP MPC solve failed: status=%s",
                  work->info ? work->info->status : "unknown");
    }
  }

  if (work) {
    osqp_cleanup(work);
  }
  c_free(p_csc);
  c_free(a_csc);

  return success && optimal_velocities.size() == static_cast<size_t>(horizon);
}

void MpcPlannerNode::fillJointTrajectoryPoint(
    const Eigen::Matrix<double, 6, 1>& positions,
    const Eigen::Matrix<double, 6, 1>& velocities,
    double time_from_start,
    trajectory_msgs::msg::JointTrajectoryPoint& point) const {
  point.positions.resize(joint_names_.size());
  point.velocities.resize(joint_names_.size());

  // 输出顺序必须和 joint_names_ 一致，否则控制器会把轨迹发给错误关节。
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    point.positions[i] = positions[static_cast<int>(i)];
    point.velocities[i] = velocities[static_cast<int>(i)];
  }

  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
}

Eigen::Matrix<double, 6, 1> MpcPlannerNode::clampJointVelocity(const Eigen::VectorXd& q_dot) const {
  Eigen::Matrix<double, 6, 1> clamped;

  for (int i = 0; i < 6; ++i) {
    clamped[i] = std::clamp(q_dot[i], -max_joint_velocity_, max_joint_velocity_);
  }

  return clamped;
}

void MpcPlannerNode::clampToJointLimits(Eigen::Matrix<double, 6, 1>& q) const {
  if (!joint_model_group_) {
    return;
  }

  // 对积分后的轨迹点再做限位裁剪，避免数值误差造成越界命令。
  const auto& joint_models = joint_model_group_->getActiveJointModels();
  for (size_t i = 0; i < joint_models.size() && i < 6; ++i) {
    const auto& bounds = joint_models[i]->getVariableBounds(joint_models[i]->getName());
    if (std::isfinite(bounds.min_position_) && std::isfinite(bounds.max_position_) &&
        bounds.min_position_ < bounds.max_position_) {
      q[static_cast<int>(i)] = std::clamp(
          q[static_cast<int>(i)], bounds.min_position_, bounds.max_position_);
    }
  }
}

void MpcPlannerNode::executeFallback() {
  if (!has_joint_state_ && (!has_last_trajectory_ || last_trajectory_.points.empty())) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "No joint state or previous trajectory available; skip fallback command");
    return;
  }

  trajectory_msgs::msg::JointTrajectory stop_traj;
  stop_traj.header.stamp.sec = 0;
  stop_traj.header.stamp.nanosec = 0;
  stop_traj.joint_names = joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint stop_point;
  Eigen::Matrix<double, 6, 1> hold_positions = current_joint_positions_;
  Eigen::Matrix<double, 6, 1> zero_velocities;
  zero_velocities.setZero();

  // 如果当前 JointState 不可用，则用上一条轨迹的第一个点作为保持位置。
  if (!has_joint_state_ && has_last_trajectory_ && !last_trajectory_.points.empty()) {
    for (size_t i = 0; i < joint_names_.size() && i < last_trajectory_.points.front().positions.size(); ++i) {
      hold_positions[static_cast<int>(i)] = last_trajectory_.points.front().positions[i];
    }
  }

  fillJointTrajectoryPoint(hold_positions, zero_velocities, config_.dt, stop_point);
  stop_traj.points.push_back(stop_point);
  nominal_traj_pub_->publish(stop_traj);
  traj_pub_->publish(stop_traj);
}

}  // namespace dynamic_assembly

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_assembly::MpcPlannerNode)
