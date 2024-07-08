cmake_minimum_required(VERSION 3.8)
project(ros2_sim_gateway)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(websocketpp REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(include)

add_executable(joint_states_listener src/joint_states_listener.cpp)
ament_target_dependencies(joint_states_listener rclcpp sensor_msgs websocketpp jsoncpp)

# Adding WebSocket++ dependencies
target_include_directories(joint_states_listener PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/websocketpp
)

target_link_libraries(joint_states_listener
  pthread
  boost_system
  jsoncpp
)

install(TARGETS
  joint_states_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
