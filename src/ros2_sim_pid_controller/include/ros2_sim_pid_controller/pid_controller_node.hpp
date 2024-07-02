#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>

class PIDControllerNode : public rclcpp::Node {
public:
    PIDControllerNode();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void setDesiredPositionsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void update();

    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd q_desired_;
    Eigen::VectorXd integral_;
    Eigen::VectorXd previous_error_;

    Eigen::VectorXd kp_;
    Eigen::VectorXd ki_;
    Eigen::VectorXd kd_;
    double max_torque_;
    Eigen::VectorXd max_integral_;
    Eigen::VectorXd max_derivative_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_positions_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    double dt_;
};

#endif // PID_CONTROLLER_NODE_HPP
