from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    
    robot_description_path = os.path.join(
        get_package_share_directory('ros2_sim_control'),
        'urdf',
        'robot.urdf'
    )

    robot_description = {"robot_description": open(robot_description_path).read()}

    send_trajectory_node = Node(
        package="ros2_sim_control",
        executable="send_trajectory",
        name="send_trajectory_node",
        parameters=[robot_description],
    )

    nodes_to_start = [send_trajectory_node]
    return LaunchDescription(nodes_to_start)