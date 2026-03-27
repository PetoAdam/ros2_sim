cmake_minimum_required(VERSION 3.8)
project(ros2_sim_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_sim_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# Command listener executable
add_executable(command_listener_node src/command_listener_node.cpp)
target_include_directories(command_listener_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(command_listener_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  moveit_msgs
  ros2_sim_msgs
  tf2
  tf2_eigen
)
target_link_libraries(command_listener_node
  moveit_ros_planning_interface::moveit_move_group_interface
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
target_include_directories(motion_planner_tester_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(motion_planner_tester_node
  rclcpp
  sensor_msgs
  moveit_msgs
)
target_link_libraries(motion_planner_tester_node
  moveit_ros_planning_interface::moveit_move_group_interface
  Eigen3::Eigen
)
install(TARGETS motion_planner_tester_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pick_and_place_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_and_place_demo
)

install(PROGRAMS
  scripts/welding_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME welding_demo
)

ament_package()
