#ifndef ROS2_SIM_CONTROL_ROBOT_CONTROLLER_HPP_
#define ROS2_SIM_CONTROL_ROBOT_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <memory>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace ros2_sim_control
{
class RobotController : public controller_interface::ControllerInterface
{
public:
  RobotController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
  trajectory_msgs::msg::JointTrajectoryPoint point_interp_;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  bool new_msg_;
  rclcpp::Time start_time_;

  // Add joint_names_ member variable
  std::vector<std::string> joint_names_;
};

}  // namespace ros2_sim_control

#endif  // ROS2_SIM_CONTROL_ROBOT_CONTROLLER_HPP_
