from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model')
    moveit_config = (
        MoveItConfigsBuilder("ros2_sim")
        .robot_description(file_path=get_package_share_directory('ros2_sim_moveit_config')
                           + "/config/{}.urdf.xacro".format(robot_model.perform(context)))
        .robot_description_semantic(get_package_share_directory('ros2_sim_moveit_config')
                                    + "/config/{}.srdf".format(robot_model.perform(context)))
        .robot_description_kinematics(file_path=get_package_share_directory('ros2_sim_moveit_config') + "/config/kinematics.yaml")
        .trajectory_execution(file_path=get_package_share_directory('ros2_sim_moveit_config') + "/config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(file_path=get_package_share_directory('ros2_sim_moveit_config')
                      + "/config/joint_limits.yaml")
        .to_moveit_configs()
    )
    
    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    command_listener_node = Node(
        package="ros2_sim_motion_planner",
        executable="command_listener_node",
        output="screen",
    )

    to_start = [
        move_group_server,
        command_listener_node,
    ]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'robot_model',
        default_value='ur3'
    ))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
