from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')

    robot_description_path = os.path.join(
        get_package_share_directory('ros2_sim_ur3_description'),
        'urdf',
        'robot.urdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_robot_state_publisher',
            default_value='true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            condition=IfCondition(use_robot_state_publisher),
            parameters=[{'robot_description': open(robot_description_path).read()}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('ros2_sim_ur3_description'), 'rviz', 'robot.rviz')],
            parameters=[{'robot_description': open(robot_description_path).read()}]
        )
    ])
