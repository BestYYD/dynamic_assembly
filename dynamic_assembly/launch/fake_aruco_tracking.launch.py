import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _load_launch_defaults():
    params_path = os.path.join(
        get_package_share_directory("dynamic_assembly"),
        "config",
        "params.yaml",
    )
    with open(params_path, "r", encoding="utf-8") as params_file:
        params = yaml.safe_load(params_file) or {}
    return params.get("ekf_camera_aruco_launch", {}).get("ros__parameters", {})



def _load_node_defaults(node_name):
    params_path = os.path.join(
        get_package_share_directory('dynamic_assembly'),
        'config',
        'params.yaml',
    )
    with open(params_path, 'r', encoding='utf-8') as params_file:
        params = yaml.safe_load(params_file) or {}
    return params.get(node_name, {}).get('ros__parameters', {})
def _launch_default(params, name, fallback):
    value = params.get(name, fallback)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def generate_launch_description():
    launch_defaults = _load_launch_defaults()
    target_defaults = _load_node_defaults('ekf_filter_node')
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()

    params_file = PathJoinSubstitution([
        FindPackageShare("dynamic_assembly"),
        "config",
        "params.yaml",
    ])

    use_fake_aruco = LaunchConfiguration("use_fake_aruco")
    use_aruco = LaunchConfiguration("use_aruco")
    use_apf = LaunchConfiguration("use_apf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    aruco_markers_topic = LaunchConfiguration("aruco_markers_topic")
    tracking_frame = LaunchConfiguration("tracking_frame")
    target_marker_id = LaunchConfiguration("target_marker_id")
    target_plane_z = LaunchConfiguration("target_plane_z")

    fake_aruco_node = Node(
        package="dynamic_assembly",
        executable="fake_aruco_publisher_node",
        name="fake_aruco_publisher_node",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "topic": aruco_markers_topic,
            "frame_id": tracking_frame,
            "marker_id": ParameterValue(target_marker_id, value_type=int),
            "rate_hz": ParameterValue(LaunchConfiguration("fake_rate_hz"), value_type=float),
            "center_x": ParameterValue(LaunchConfiguration("fake_center_x"), value_type=float),
            "center_y": ParameterValue(LaunchConfiguration("fake_center_y"), value_type=float),
            "center_z": ParameterValue(target_plane_z, value_type=float),
            "amplitude_x": ParameterValue(LaunchConfiguration("fake_amplitude_x"), value_type=float),
            "amplitude_y": ParameterValue(LaunchConfiguration("fake_amplitude_y"), value_type=float),
            "omega_x": ParameterValue(LaunchConfiguration("fake_omega_x"), value_type=float),
            "omega_y": ParameterValue(LaunchConfiguration("fake_omega_y"), value_type=float),
            "phase_y": ParameterValue(LaunchConfiguration("fake_phase_y"), value_type=float),
            "yaw": ParameterValue(LaunchConfiguration("fake_yaw"), value_type=float),
        }],
        condition=IfCondition(use_fake_aruco),
    )

    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[{
            "marker_size": ParameterValue(LaunchConfiguration("marker_size"), value_type=float),
            "aruco_dictionary_id": LaunchConfiguration("aruco_dictionary_id"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        }],
        remappings=[("aruco_markers", aruco_markers_topic)],
        condition=IfCondition(use_aruco),
    )

    ekf_node = Node(
        package="dynamic_assembly",
        executable="ekf_filter_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "aruco_markers": aruco_markers_topic,
                "target_marker_id": ParameterValue(target_marker_id, value_type=int),
                "target_plane_z": ParameterValue(target_plane_z, value_type=float),
                "tracking_frame": tracking_frame,
                "use_tf_transform": ParameterValue(LaunchConfiguration("use_tf_transform"), value_type=bool),
            },
        ],
    )

    apf_node = Node(
        package="dynamic_assembly",
        executable="apf_planner_node",
        name="apf_planner_node",
        output="screen",
        parameters=[
            params_file,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "target_plane_z": ParameterValue(target_plane_z, value_type=float),
                "frame_id": tracking_frame,
            },
        ],
        condition=IfCondition(use_apf),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_aruco", default_value="true", description="Start fake /aruco_markers publisher."),
        DeclareLaunchArgument("use_aruco", default_value="false", description="Start real ros2_aruco. Do not enable with fake on the same topic."),
        DeclareLaunchArgument("use_apf", default_value="true", description="Start APF after EKF."),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use Gazebo simulation time."),
        DeclareLaunchArgument("use_tf_transform", default_value="true", description="Transform ArUco poses into tracking_frame in EKF."),
        DeclareLaunchArgument("aruco_markers_topic", default_value="/aruco_markers", description="Aruco marker topic consumed by EKF."),
        DeclareLaunchArgument("tracking_frame", default_value="world", description="Frame used by fake ArUco and EKF output."),
        DeclareLaunchArgument("target_marker_id", default_value="0", description="Marker id tracked by EKF."),
        DeclareLaunchArgument("target_plane_z", default_value=_launch_default(target_defaults, 'target_plane_z', 0.25), description="Fixed target plane height."),
        DeclareLaunchArgument("fake_start_delay", default_value="0.5", description="Delay before fake ArUco starts."),
        DeclareLaunchArgument("aruco_start_delay", default_value="0.5", description="Delay before real ArUco starts."),
        DeclareLaunchArgument("ekf_start_delay", default_value="1.0", description="Delay before EKF starts."),
        DeclareLaunchArgument("apf_start_delay", default_value="2.0", description="Delay before APF starts."),
        DeclareLaunchArgument("fake_rate_hz", default_value="30.0", description="Fake marker publish rate."),
        DeclareLaunchArgument("fake_center_x", default_value="0.43", description="Fake marker center x."),
        DeclareLaunchArgument("fake_center_y", default_value="0.05", description="Fake marker center y."),
        DeclareLaunchArgument("fake_amplitude_x", default_value="0.035", description="Fake marker x amplitude."),
        DeclareLaunchArgument("fake_amplitude_y", default_value="0.035", description="Fake marker y amplitude."),
        DeclareLaunchArgument("fake_omega_x", default_value="0.4", description="Fake marker x angular speed."),
        DeclareLaunchArgument("fake_omega_y", default_value="0.4", description="Fake marker y angular speed."),
        DeclareLaunchArgument("fake_phase_y", default_value="1.5708", description="Fake marker y phase."),
        DeclareLaunchArgument("fake_yaw", default_value="0.0", description="Fake marker yaw."),
        DeclareLaunchArgument("marker_size", default_value=_launch_default(launch_defaults, "marker_size", 0.10), description="Real ArUco marker size."),
        DeclareLaunchArgument("aruco_dictionary_id", default_value=_launch_default(launch_defaults, "aruco_dictionary_id", "DICT_6X6_50"), description="Real ArUco dictionary."),
        DeclareLaunchArgument("image_topic", default_value=_launch_default(launch_defaults, "image_topic", "/image_raw"), description="Image topic for real ArUco."),
        DeclareLaunchArgument("camera_info_topic", default_value=_launch_default(launch_defaults, "camera_info_topic", "/camera_info"), description="Camera info topic for real ArUco."),
        TimerAction(period=LaunchConfiguration("fake_start_delay"), actions=[fake_aruco_node], condition=IfCondition(use_fake_aruco)),
        TimerAction(period=LaunchConfiguration("aruco_start_delay"), actions=[aruco_node], condition=IfCondition(use_aruco)),
        TimerAction(period=LaunchConfiguration("ekf_start_delay"), actions=[ekf_node]),
        TimerAction(period=LaunchConfiguration("apf_start_delay"), actions=[apf_node], condition=IfCondition(use_apf)),
    ])
