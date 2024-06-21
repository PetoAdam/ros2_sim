#include "ros2_sim_control/robot_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace ros2_sim_control
{
RobotController::RobotController()
: controller_interface::ControllerInterface(), new_msg_(false) {}

controller_interface::CallbackReturn RobotController::on_init()
{
  joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      trajectory_msg_ = msg;
      new_msg_ = true;
      // Initialize joint_names_ based on received message
      joint_names_.clear();
      for (const auto& joint : msg->joint_names) {
        joint_names_.push_back(joint);
      }
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/position");
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/position");
  }

  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  if (!trajectory_msg_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  start_time_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  trajectory_msg_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
{
  trajectory_msg_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (new_msg_) {
    new_msg_ = false;
    start_time_ = time;
  }

  if (trajectory_msg_) {
    // Implement your control algorithm here, e.g., PID control
    // Use trajectory_msg_ and current joint states to compute commands
    // For demonstration, we'll print the received trajectory point
    RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory point!");
  }

  return controller_interface::return_type::OK;
}

}  // namespace ros2_sim_control

PLUGINLIB_EXPORT_CLASS(ros2_sim_control::RobotController, controller_interface::ControllerInterface)
