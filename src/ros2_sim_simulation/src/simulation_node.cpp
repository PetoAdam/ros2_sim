#include "ros2_sim_simulation/simulation_node.hpp"
#include <cmath>

SimulationNode::SimulationNode() : Node("ros2_sim_simulation_node") {
    // Declare and get parameters
    declare_parameter("urdf_path", "");
    declare_parameter("dt", 0.005);
    declare_parameter("joint_viscous_friction", 0.01);
    declare_parameter("damping_factor", -0.01);

    get_parameter("urdf_path", urdf_path_);
    get_parameter("dt", dt_);
    get_parameter("joint_viscous_friction", joint_viscous_friction_);
    double damping_factor = 0.0;
    get_parameter("damping_factor", damping_factor);
    if (joint_viscous_friction_ == 0.0 && damping_factor != 0.0) {
        joint_viscous_friction_ = std::abs(damping_factor);
    }

    simulation_steps_ = 0;

    publisher_ = create_publisher<sensor_msgs::msg::JointState>("sim_joint_states", 0);
    torque_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
        "torques", 0, std::bind(&SimulationNode::torqueCallback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Empty>(
        "reset_simulation",
        std::bind(&SimulationNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

    pinocchio::urdf::buildModel(urdf_path_, model_);
    data_ = pinocchio::Data(model_);

    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    a_ = Eigen::VectorXd::Zero(model_.nv);
    tau_ = Eigen::VectorXd::Zero(model_.nv);
    external_tau_ = Eigen::VectorXd::Zero(model_.nv);

    joint_names_.reserve(model_.njoints - 1);
    for (size_t i = 1; i < model_.names.size(); ++i) {
        if (model_.names[i].find("joint") != std::string::npos) {
            joint_names_.push_back(model_.names[i]);
        }
    }

    q_desired_ = Eigen::VectorXd::Zero(model_.nq);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt_ * 1000)), std::bind(&SimulationNode::update, this));
}

void SimulationNode::reset() {
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    a_ = Eigen::VectorXd::Zero(model_.nv);
    tau_ = Eigen::VectorXd::Zero(model_.nv);
    external_tau_ = Eigen::VectorXd::Zero(model_.nv);

    simulation_steps_ = 0;
}

void SimulationNode::update() {

    auto start_time = std::chrono::high_resolution_clock::now();

    // Apply joint viscous friction
    tau_ = -joint_viscous_friction_ * v_;

    // Add external torque from PID controller
    tau_ += external_tau_;

    // Update the robot's state based on the current torques and dynamics
    a_ = pinocchio::aba(model_, data_, q_, v_, tau_);
    v_ += a_ * dt_;
    q_ = pinocchio::integrate(model_, q_, v_ * dt_);

    for (int i = 0; i < q_.size(); ++i) {
            q_[i] = std::clamp(q_[i], model_.lowerPositionLimit[i], model_.upperPositionLimit[i]);
    }

    if (q_.hasNaN() || v_.hasNaN() || a_.hasNaN() || tau_.hasNaN()) {
        RCLCPP_WARN(this->get_logger(), "NaN detected in joint state. Resetting simulation.");
        reset();
        return;
    }

    // Publish the updated joint states
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = now();
    joint_state_msg.name = joint_names_;
    joint_state_msg.position.resize(q_.size());
    joint_state_msg.velocity.resize(v_.size());

    Eigen::VectorXd::Map(&joint_state_msg.position[0], q_.size()) = q_;
    Eigen::VectorXd::Map(&joint_state_msg.velocity[0], v_.size()) = v_;

    publisher_->publish(joint_state_msg);

    // Increment simulation step count
    simulation_steps_++;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    auto sleep_time = std::chrono::milliseconds(static_cast<int>(dt_ * 1000)) - std::chrono::milliseconds(elapsed_time);

    if (sleep_time.count() > 0) {
        std::this_thread::sleep_for(sleep_time);
    }
}

void SimulationNode::torqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Handle incoming torques
    if (msg->effort.size() == static_cast<size_t>(external_tau_.size())) {
        Eigen::VectorXd::Map(&external_tau_[0], external_tau_.size()) = Eigen::VectorXd::Map(msg->effort.data(), msg->effort.size());
    }
}

void SimulationNode::resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
    RCLCPP_INFO(this->get_logger(), "Reset service called. Resetting simulation state.");
    reset();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
