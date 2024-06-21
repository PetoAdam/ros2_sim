#ifndef SIMULATION_NODE_HPP
#define SIMULATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <Eigen/Dense>

class SimulationNode : public rclcpp::Node {
public:
    SimulationNode();

private:
    void reset();
    void update();
    void torqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Parameters from config file
    std::string urdf_path_;
    double dt_;
    double damping_factor_;

    int simulation_steps_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torque_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    pinocchio::Model model_;
    pinocchio::Data data_;
    std::vector<std::string> joint_names_;

    Eigen::VectorXd q_, v_, a_, tau_, external_tau_;
    Eigen::VectorXd q_desired_;
};

#endif // SIMULATION_NODE_HPP