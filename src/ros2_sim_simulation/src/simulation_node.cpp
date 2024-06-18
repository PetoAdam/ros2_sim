#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <thread>

class SimulationNode : public rclcpp::Node {
public:
    SimulationNode() : Node("ros2_sim_simulation_node") {

        // Declare parameters (actually does not matter what we initialize inside them)
        declare_parameter("urdf_path", "/home/ws/src/ros2_sim_ur3_description/urdf/robot.urdf");
        declare_parameter("damping_factor", 0.0);
        declare_parameter("dt", 0.0);

        // Get parameters from the config file
        urdf_path = get_parameter("urdf_path").as_string();
        damping_factor = get_parameter("damping_factor").as_double();
        dt = get_parameter("dt").as_double();

        RCLCPP_INFO(this->get_logger(), "URDF path: %s", urdf_path.c_str());
        RCLCPP_INFO(this->get_logger(), "damping factor: %f", damping_factor);
        RCLCPP_INFO(this->get_logger(), "Time step (dt): %f", dt);

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Load the URDF model
        pinocchio::urdf::buildModel(urdf_path, model);
        data = pinocchio::Data(model);

        // Initialize joint states
        q = Eigen::VectorXd::Zero(model.nq);
        v = Eigen::VectorXd::Zero(model.nv);
        tau = Eigen::VectorXd::Zero(model.nv);

        // Extract actual joint names
        joint_names.reserve(model.njoints - 1);  // excluding the universe joint
        for (size_t i = 1; i < model.names.size(); ++i) {
            if (model.names[i].find("joint") != std::string::npos) {
                joint_names.push_back(model.names[i]);
            }
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt * 1000)),
            std::bind(&SimulationNode::update, this)
        );
    }

private:
    void update() {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Apply damping
        tau += damping_factor * v;

        // Compute forward dynamics
        Eigen::VectorXd a = pinocchio::aba(model, data, q, v, tau);

        // Integrate to get new velocity and position
        v += a * dt;
        q = pinocchio::integrate(model, q, v * dt);

        // Clamp joint positions within limits
        for (int i = 0; i < q.size(); ++i) {
            q[i] = std::clamp(q[i], model.lowerPositionLimit[i], model.upperPositionLimit[i]);
        }

        // Publish joint states
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = joint_names;
        joint_state.position = std::vector<double>(q.data(), q.data() + q.size());
        joint_state.velocity = std::vector<double>(v.data(), v.data() + v.size());
        joint_state.effort.resize(v.size(), 0.0); // Assuming zero effort for simplicity

        publisher_->publish(joint_state);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        auto sleep_time = std::chrono::milliseconds(static_cast<int>(dt * 1000)) - std::chrono::milliseconds(elapsed_time);

        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    std::string urdf_path;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    pinocchio::Model model;
    pinocchio::Data data;
    std::vector<std::string> joint_names;

    Eigen::VectorXd q, v, tau;
    double dt;
    double damping_factor;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulationNode>());
    rclcpp::shutdown();
    return 0;
}
