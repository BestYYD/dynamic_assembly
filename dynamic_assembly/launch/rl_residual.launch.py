from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    enabled = LaunchConfiguration("enabled")
    policy_mode = LaunchConfiguration("policy_mode")
    publish_corrected_to_controller = LaunchConfiguration("publish_corrected_to_controller")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("dynamic_assembly"),
                "config",
                "params.yaml",
            ]),
            description="Parameter file for the RL residual framework node.",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="false",
            description="Enable residual action output. False keeps zero residuals.",
        ),
        DeclareLaunchArgument(
            "policy_mode",
            default_value="zero",
            description="Residual policy mode: zero or topic.",
        ),
        DeclareLaunchArgument(
            "publish_corrected_to_controller",
            default_value="false",
            description="Publish corrected command to the real controller topic.",
        ),
        Node(
            package="dynamic_assembly",
            executable="rl_residual_node",
            name="rl_residual_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "enabled": ParameterValue(enabled, value_type=bool),
                    "policy_mode": policy_mode,
                    "publish_corrected_to_controller": ParameterValue(
                        publish_corrected_to_controller, value_type=bool),
                },
            ],
        ),
    ])

