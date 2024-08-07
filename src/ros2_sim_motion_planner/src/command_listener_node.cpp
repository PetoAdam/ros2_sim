#include "ros2_sim_motion_planner/command_listener_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CommandListenerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
