from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 获取 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 启动 RViz, TF, 机器人状态发布器等
    ar4_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ar4_gazebo_bringup"),
                "launch",
                "main.launch.py"
            ])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    use_camera = LaunchConfiguration("use_camera")
    use_aruco = LaunchConfiguration("use_aruco")

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "pixel_format": LaunchConfiguration("camera_pixel_format"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
            "camera_name": LaunchConfiguration("camera_name"),
        }],
        condition=IfCondition(use_camera),
    )

    # Aruco 识别节点：等价于用户原来的 ros2 run ros2_aruco aruco_node ... 命令。
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "marker_size": ParameterValue(LaunchConfiguration("marker_size"), value_type=float),
            "aruco_dictionary_id": LaunchConfiguration("aruco_dictionary_id"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        }],
    )

    # 摄像头先起来一点时间，再启动 Aruco，避免 Aruco 一启动就收不到 camera_info。
    aruco_start = TimerAction(
        period=3.0,
        actions=[aruco_node],
        condition=IfCondition(use_aruco),
    )

    # 多线程规划系统
    assembly_system_node = Node(
        package="dynamic_assembly",
        executable="assembly_system",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            PathJoinSubstitution([
                FindPackageShare("dynamic_assembly"),
                "config",
                "params.yaml"
            ]),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # assembly_system 内含 EKF/APF/MPC，稍微延后启动，让相机和 Aruco 先进入发布状态。
    assembly_system_start = TimerAction(
        period=1.5,
        actions=[assembly_system_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock for all dynamic assembly nodes."
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Start usb_cam before Aruco."
        ),
        DeclareLaunchArgument(
            "use_aruco",
            default_value="true",
            description="Start ros2_aruco and feed /aruco_markers into the EKF."
        ),
        DeclareLaunchArgument(
            "camera_pixel_format",
            default_value="mjpeg2rgb",
            description="usb_cam pixel_format parameter."
        ),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value="file:///home/bran_24/camera_config/my_usb_cam.yaml",
            description="usb_cam camera calibration file."
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="default_cam",
            description="usb_cam camera name; should match the calibration YAML camera_name."
        ),
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.10",
            description="Aruco marker size in meters."
        ),
        DeclareLaunchArgument(
            "aruco_dictionary_id",
            default_value="DICT_6X6_50",
            description="Aruco dictionary id."
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/image_raw",
            description="Image topic used by ros2_aruco."
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera_info",
            description="Camera info topic used by ros2_aruco."
        ),
        camera_node,
        aruco_start,
        ar4_base_launch,
        assembly_system_start
    ])
