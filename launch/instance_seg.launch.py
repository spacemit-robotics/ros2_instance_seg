"""Launch the instance_seg node with its default ROS parameters."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("instance_seg"), "config", "instance_seg.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/image_raw"
            ),
            DeclareLaunchArgument("score_threshold", default_value="0.25"),
            Node(
                package="instance_seg",
                executable="instance_seg_node",
                name="instance_seg_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_camera": LaunchConfiguration("use_camera"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "score_threshold": LaunchConfiguration("score_threshold"),
                    },
                ],
            ),
        ]
    )
