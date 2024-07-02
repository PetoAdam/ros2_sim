from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the MoveIt planning pipeline
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group_node',
            output='screen',
            parameters=[{'planning_plugin': 'ompl_interface/OMPLPlanner'}],  # Adjust the planner as needed
        ),
        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'config/moveit.rviz'],  # Specify the path to your RViz configuration file
        ),
    ])
