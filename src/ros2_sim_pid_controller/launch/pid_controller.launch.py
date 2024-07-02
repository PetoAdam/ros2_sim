from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the pid_controller_config.yaml file
    config_file = os.path.join(
        get_package_share_directory('ros2_sim_pid_controller'),
        'config',
        'pid_controller_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_sim_pid_controller',
            executable='ros2_sim_pid_controller_node',
            name='ros2_sim_pid_controller_node',
            output='screen',
            parameters=[config_file],  # Pass as a list
        ),
    ])
