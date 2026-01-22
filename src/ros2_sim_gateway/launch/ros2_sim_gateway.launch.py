import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('ros2_sim_ur3_description'),
        'urdf',
        'robot.urdf'
    )

    return LaunchDescription([
        Node(
            package='ros2_sim_gateway',
            executable='joint_states_listener',
            name='joint_states_listener',
            output='screen',
            parameters=[
                # Add any parameters if needed
            ]
        ),
        Node(
            package='ros2_sim_gateway',
            executable='robot_description_publisher',
            name='robot_description_publisher',
            output='screen',
            parameters=[
                {'robot_description_path': robot_description_path}
            ]
        )
    ])
