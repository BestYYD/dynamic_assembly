#include <rclcpp/rclcpp.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

class FakeArucoPublisherNode : public rclcpp::Node {
public:
  FakeArucoPublisherNode() : Node("fake_aruco_publisher_node") {
    if (!has_parameter("use_sim_time")) { declare_parameter<bool>("use_sim_time", true); }
    topic_ = declare_parameter<std::string>("topic", "/aruco_markers");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    marker_id_ = declare_parameter<int>("marker_id", 0);
    rate_hz_ = declare_parameter<double>("rate_hz", 30.0);
    center_x_ = declare_parameter<double>("center_x", 0.43);
    center_y_ = declare_parameter<double>("center_y", 0.05);
    center_z_ = declare_parameter<double>("center_z", 0.12);
    amplitude_x_ = declare_parameter<double>("amplitude_x", 0.035);
    amplitude_y_ = declare_parameter<double>("amplitude_y", 0.035);
    omega_x_ = declare_parameter<double>("omega_x", 0.4);
    omega_y_ = declare_parameter<double>("omega_y", 0.4);
    phase_y_ = declare_parameter<double>("phase_y", 1.5708);
    yaw_ = declare_parameter<double>("yaw", 0.0);

    publisher_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>(topic_, 10);
    start_time_ = std::chrono::steady_clock::now();
    const double safe_rate = std::max(1.0, rate_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_rate));
    timer_ = create_wall_timer(period, std::bind(&FakeArucoPublisherNode::publishMarker, this));

    RCLCPP_INFO(get_logger(), "Fake ArUco publisher: topic=%s frame=%s marker_id=%d rate=%.1fHz",
                topic_.c_str(), frame_id_.c_str(), marker_id_, safe_rate);
  }

private:
  void publishMarker() {
    const double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();

    ros2_aruco_interfaces::msg::ArucoMarkers msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.marker_ids.push_back(marker_id_);

    geometry_msgs::msg::Pose pose;
    pose.position.x = center_x_ + amplitude_x_ * std::sin(omega_x_ * t);
    pose.position.y = center_y_ + amplitude_y_ * std::sin(omega_y_ * t + phase_y_);
    pose.position.z = center_z_;
    pose.orientation.z = std::sin(0.5 * yaw_);
    pose.orientation.w = std::cos(0.5 * yaw_);
    msg.poses.push_back(pose);

    publisher_->publish(msg);
  }

  std::string topic_;
  std::string frame_id_;
  int marker_id_ = 0;
  double rate_hz_ = 30.0;
  double center_x_ = 0.43;
  double center_y_ = 0.05;
  double center_z_ = 0.12;
  double amplitude_x_ = 0.035;
  double amplitude_y_ = 0.035;
  double omega_x_ = 0.4;
  double omega_y_ = 0.4;
  double phase_y_ = 1.5708;
  double yaw_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeArucoPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
