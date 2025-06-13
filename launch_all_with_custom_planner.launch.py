from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    simulation_launch = os.path.join(
        get_package_share_directory('ros2_sim_simulation'),
        'launch', 'simulation.launch.py'
    )
    pid_controller_launch = os.path.join(
        get_package_share_directory('ros2_sim_pid_controller'),
        'launch', 'pid_controller.launch.py'
    )
    control_launch = os.path.join(
        get_package_share_directory('ros2_sim_control'),
        'launch', 'ros2_sim_control.launch.py'
    )
    gateway_launch = os.path.join(
        get_package_share_directory('ros2_sim_gateway'),
        'launch', 'ros2_sim_gateway.launch.py'
    )
    planner_launch = os.path.join(
        get_package_share_directory('ros2_sim_motion_planner'),
        'launch', 'motion_planner.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(simulation_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pid_controller_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gateway_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(planner_launch)),
    ])