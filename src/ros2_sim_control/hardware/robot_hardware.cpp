#include "ros2_sim_control/robot_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ros2_sim_control
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joint_positions_.assign(6, 0);
  joint_commands_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  node_ = rclcpp::Node::make_shared("robot_hardware_node");

  desired_positions_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "desired_positions", 0);

  joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 0, std::bind(&RobotSystem::jointStateCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_positions_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_commands_[ind++]);
  }

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  rclcpp::spin_some(node_);
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto desired_positions_msg = std::make_shared<sensor_msgs::msg::JointState>();
  int ind = 0;
  for (const auto & joint : info_.joints) {
    desired_positions_msg->name.push_back(joint.name);
    desired_positions_msg->position.push_back(joint_commands_[ind++]);
  }
  desired_positions_publisher_->publish(*desired_positions_msg);
  return return_type::OK;
}

void RobotSystem::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    joint_positions_[i] = msg->position[i];
  }
}

}  // namespace ros2_sim_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_sim_control::RobotSystem, hardware_interface::SystemInterface)
