import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_sim_gateway',
            executable='joint_states_listener',
            name='joint_states_listener',
            output='screen',
            parameters=[
                # Add any parameters if needed
            ]
        )
    ])
