"""Launch SABO."""
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    
    interface_node = Node(
        package='follow_human_pkg',
        executable='leg_finder',
        respawn=True
    )

    ld = LaunchDescription()
    ld.add_action(interface_node)
    return ld
