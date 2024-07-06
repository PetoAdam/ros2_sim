cmake_minimum_required(VERSION 3.8)
project(ros2_sim_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_sim_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# include directories
include_directories(
  include
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Command listener executable
add_executable(command_listener_node src/command_listener_node.cpp)
ament_target_dependencies(command_listener_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
  ros2_sim_msgs
  tf2
  tf2_eigen
  Eigen3
)
target_link_libraries(command_listener_node
  tf2::tf2
  tf2_eigen::tf2_eigen
  Eigen3::Eigen
)
install(TARGETS
  command_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

# Executable for testing
add_executable(motion_planner_tester_node src/motion_planner_tester.cpp)
ament_target_dependencies(motion_planner_tester_node
  rclcpp
  sensor_msgs
  moveit_ros_planning_interface
)
target_link_libraries(motion_planner_tester_node)
install(TARGETS motion_planner_tester_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
