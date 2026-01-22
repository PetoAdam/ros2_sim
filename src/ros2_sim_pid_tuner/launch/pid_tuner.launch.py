from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("ros2_sim_pid_tuner"),
        "config",
        "pid_tuner.yaml",
    )

    return LaunchDescription([
        Node(
            package="ros2_sim_pid_tuner",
            executable="ros2_sim_pid_tuner_node",
            name="ros2_sim_pid_tuner",
            output="screen",
            parameters=[config_file],
        )
    ])
