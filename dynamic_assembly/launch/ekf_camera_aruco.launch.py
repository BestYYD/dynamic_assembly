import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _load_camera_launch_defaults():
    params_path = os.path.join(
        get_package_share_directory("dynamic_assembly"),
        "config",
        "params.yaml",
    )
    with open(params_path, "r", encoding="utf-8") as params_file:
        params = yaml.safe_load(params_file) or {}
    return params.get("ekf_camera_aruco_launch", {}).get("ros__parameters", {})


def _launch_default(params, name, fallback):
    value = params.get(name, fallback)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def generate_launch_description():
    launch_defaults = _load_camera_launch_defaults()
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()

    use_camera = LaunchConfiguration("use_camera")
    use_aruco = LaunchConfiguration("use_aruco")
    use_apf = LaunchConfiguration("use_apf")
    video_device = LaunchConfiguration("video_device")

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_camera_static_tf",
        output="screen",
        arguments=[
            "--x",
            LaunchConfiguration("camera_tf_x"),
            "--y",
            LaunchConfiguration("camera_tf_y"),
            "--z",
            LaunchConfiguration("camera_tf_z"),
            "--roll",
            LaunchConfiguration("camera_tf_roll"),
            "--pitch",
            LaunchConfiguration("camera_tf_pitch"),
            "--yaw",
            LaunchConfiguration("camera_tf_yaw"),
            "--frame-id",
            LaunchConfiguration("camera_tf_parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("camera_tf_child_frame"),
        ],
    )

    set_camera_exposure = ExecuteProcess(
        cmd=[
            "v4l2-ctl",
            "-d",
            video_device,
            "-c",
            LaunchConfiguration("camera_auto_exposure_control"),
        ],
        output="screen",
        condition=IfCondition(use_camera),
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": video_device,
            "pixel_format": LaunchConfiguration("camera_pixel_format"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
            "camera_name": LaunchConfiguration("camera_name"),
        }],
        condition=IfCondition(use_camera),
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
    )

    ekf_node = Node(
        package="dynamic_assembly",
        executable="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("dynamic_assembly"),
                "config",
                "params.yaml",
            ]),
        ],
    )

    apf_node = Node(
        package="dynamic_assembly",
        executable="apf_planner_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("dynamic_assembly"),
                "config",
                "params.yaml",
            ]),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    aruco_start = TimerAction(
        period=float(launch_defaults.get("aruco_start_delay", 1.0)),
        actions=[aruco_node],
        condition=IfCondition(use_aruco),
    )

    camera_start = TimerAction(
        period=float(launch_defaults.get("camera_start_delay", 0.5)),
        actions=[camera_node],
        condition=IfCondition(use_camera),
    )

    # EKF 可以先启动订阅等待消息；这里略微延后，方便日志顺序更清楚。
    ekf_start = TimerAction(
        period=float(launch_defaults.get("ekf_start_delay", 1.5)),
        actions=[ekf_node],
    )

    apf_start = TimerAction(
        period=float(launch_defaults.get("apf_start_delay", 2.0)),
        actions=[apf_node],
        condition=IfCondition(use_apf),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_camera",
            default_value=_launch_default(launch_defaults, "use_camera", True),
            description="Start usb_cam before Aruco.",
        ),
        DeclareLaunchArgument(
            "use_aruco",
            default_value=_launch_default(launch_defaults, "use_aruco", True),
            description="Start ros2_aruco and feed /aruco_markers into the EKF.",
        ),
        DeclareLaunchArgument(
            "use_apf",
            default_value=_launch_default(launch_defaults, "use_apf", True),
            description="Start APF planner after EKF; MPC is not started by this launch.",
        ),
        DeclareLaunchArgument(
            "video_device",
            default_value=_launch_default(launch_defaults, "video_device", "/dev/video0"),
            description="Video device used by usb_cam and v4l2-ctl.",
        ),
        DeclareLaunchArgument(
            "camera_auto_exposure_control",
            default_value=_launch_default(
                launch_defaults, "camera_auto_exposure_control", "auto_exposure=1"),
            description="v4l2-ctl camera exposure control assignment.",
        ),
        DeclareLaunchArgument(
            "camera_pixel_format",
            default_value=_launch_default(launch_defaults, "camera_pixel_format", "mjpeg2rgb"),
            description="usb_cam pixel_format parameter.",
        ),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value=_launch_default(
                launch_defaults,
                "camera_info_url",
                "file:///home/bran_24/camera_config/my_usb_cam.yaml"),
            description="usb_cam camera calibration file.",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value=_launch_default(launch_defaults, "camera_name", "default_cam"),
            description="usb_cam camera name; should match the calibration YAML camera_name.",
        ),
        DeclareLaunchArgument(
            "camera_tf_parent_frame",
            default_value=_launch_default(launch_defaults, "camera_tf_parent_frame", "world"),
            description="Parent frame for the camera static transform.",
        ),
        DeclareLaunchArgument(
            "camera_tf_child_frame",
            default_value=_launch_default(launch_defaults, "camera_tf_child_frame", "default_cam"),
            description="Child frame for the camera static transform.",
        ),
        DeclareLaunchArgument(
            "camera_tf_x",
            default_value=_launch_default(launch_defaults, "camera_tf_x", 0.0),
            description="Camera static transform x.",
        ),
        DeclareLaunchArgument(
            "camera_tf_y",
            default_value=_launch_default(launch_defaults, "camera_tf_y", 0.0),
            description="Camera static transform y.",
        ),
        DeclareLaunchArgument(
            "camera_tf_z",
            default_value=_launch_default(launch_defaults, "camera_tf_z", 0.5),
            description="Camera static transform z.",
        ),
        DeclareLaunchArgument(
            "camera_tf_roll",
            default_value=_launch_default(launch_defaults, "camera_tf_roll", 0.0),
            description="Camera static transform roll.",
        ),
        DeclareLaunchArgument(
            "camera_tf_pitch",
            default_value=_launch_default(launch_defaults, "camera_tf_pitch", 0.0),
            description="Camera static transform pitch.",
        ),
        DeclareLaunchArgument(
            "camera_tf_yaw",
            default_value=_launch_default(launch_defaults, "camera_tf_yaw", 0.0),
            description="Camera static transform yaw.",
        ),
        DeclareLaunchArgument(
            "marker_size",
            default_value=_launch_default(launch_defaults, "marker_size", 0.10),
            description="Aruco marker size in meters.",
        ),
        DeclareLaunchArgument(
            "aruco_dictionary_id",
            default_value=_launch_default(launch_defaults, "aruco_dictionary_id", "DICT_6X6_50"),
            description="Aruco dictionary id.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value=_launch_default(launch_defaults, "image_topic", "/image_raw"),
            description="Image topic used by ros2_aruco.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value=_launch_default(launch_defaults, "camera_info_topic", "/camera_info"),
            description="Camera info topic used by ros2_aruco.",
        ),
        camera_static_tf,
        set_camera_exposure,
        camera_start,
        aruco_start,
        ekf_start,
        apf_start,
    ])
