#ifndef ROS2_SIM_CONTROL__ROBOT_HARDWARE_HPP_
#define ROS2_SIM_CONTROL__ROBOT_HARDWARE_HPP_

#ifndef HARDWARE_INTERFACE_PUBLIC
#define HARDWARE_INTERFACE_PUBLIC
#endif

#include <vector>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace ros2_sim_control
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_positions_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_commands_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}};
};
}  // namespace ros2_sim_control

#endif  // ROS2_SIM__CONTROL_ROBOT_HARDWARE_HPP_
