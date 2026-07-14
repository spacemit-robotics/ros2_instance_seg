"""Launch the instance_seg node with its default ROS parameters."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([Node(
        package="instance_seg", executable="instance_seg_node", name="instance_seg_node",
        output="screen", parameters=[PathJoinSubstitution(
            [FindPackageShare("instance_seg"), "config", "instance_seg.yaml"])]
    )])
