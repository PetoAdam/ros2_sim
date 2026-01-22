#include "ros2_sim_pid_controller/pid_controller_node.hpp"

PIDControllerNode::PIDControllerNode() : Node("pid_controller_node") {
    // Declare and get parameters
    declare_parameter("urdf_path", "");
    declare_parameter("kp", std::vector<double>{1.7688, 6.01637, 7.5294, 7.18778, 5.87989, 4.0});
    declare_parameter("ki", std::vector<double>{1.57731, 6.34717, 6.3343, 6.17437, 7.02989, 0.0});
    declare_parameter("kd", std::vector<double>{2.57169, 7.9477, 5.98217, 5.26123, 1.61688, 0.0});
    declare_parameter("dt", 0.005);
    declare_parameter("use_gravity_ff", true);
    declare_parameter("use_coriolis_ff", false);
    declare_parameter("allow_gain_updates", true);
    declare_parameter("reset_integral_on_gain_change", true);

    std::string urdf_path;
    get_parameter("urdf_path", urdf_path);
    std::vector<double> kp, ki, kd;
    get_parameter("kp", kp);
    get_parameter("ki", ki);
    get_parameter("kd", kd);
    get_parameter("dt", dt_);
    get_parameter("use_gravity_ff", use_gravity_ff_);
    get_parameter("use_coriolis_ff", use_coriolis_ff_);
    get_parameter("allow_gain_updates", allow_gain_updates_);
    get_parameter("reset_integral_on_gain_change", reset_integral_on_gain_change_);

    // Initialize model and data
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    kp_ = Eigen::VectorXd::Map(kp.data(), kp.size());
    ki_ = Eigen::VectorXd::Map(ki.data(), ki.size());
    kd_ = Eigen::VectorXd::Map(kd.data(), kd.size());

    std::cout << "Kp: " << std::endl << kp_ << std::endl;
    std::cout << "Ki: " << std::endl << ki_ << std::endl;
    std::cout << "Kd: " << std::endl << kd_ << std::endl;

    integral_ = Eigen::VectorXd::Zero(model_.nq);
    previous_error_ = Eigen::VectorXd::Zero(model_.nv);
    q_desired_ = Eigen::VectorXd::Zero(model_.nq);

    max_torque_ = 100.0;
    max_integral_ = Eigen::VectorXd::Constant(model_.nq, 100.0);
    max_derivative_ = Eigen::VectorXd::Constant(model_.nq, 100.0);

    // Publishers and subscribers
    torque_publisher_ = create_publisher<sensor_msgs::msg::JointState>("torques", 0);
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
        "sim_joint_states", 0, std::bind(&PIDControllerNode::jointStateCallback, this, std::placeholders::_1));
    desired_positions_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
        "desired_positions", 0, std::bind(&PIDControllerNode::setDesiredPositionsCallback, this, std::placeholders::_1));

    if (allow_gain_updates_) {
        params_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&PIDControllerNode::onSetParameters, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)), std::bind(&PIDControllerNode::update, this));
}

void PIDControllerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Handle incoming joint states
    if (msg->position.size() == static_cast<size_t>(model_.nq) && msg->velocity.size() == static_cast<size_t>(model_.nv)) {
        Eigen::VectorXd q = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
        Eigen::VectorXd v = Eigen::VectorXd::Map(msg->velocity.data(), msg->velocity.size());

        Eigen::VectorXd feedforward = Eigen::VectorXd::Zero(model_.nv);
        if (use_gravity_ff_) {
            pinocchio::computeGeneralizedGravity(model_, data_, q);
            feedforward += data_.g;
        }
        if (use_coriolis_ff_) {
            pinocchio::computeCoriolisMatrix(model_, data_, q, v);
            feedforward += data_.C * v;
        }
        
        // Compute PID control
        Eigen::VectorXd error = q_desired_ - q;
        Eigen::VectorXd proportional = kp_.array() * error.array();

        // Optional logging
        //std::cout << "Q: " << std::endl << q << std::endl << "Q desired: " << std::endl << q_desired_ << std::endl << "Error: " << std::endl << error << std::endl;

        integral_ += error * dt_;
        integral_ = integral_.cwiseMin(max_integral_).cwiseMax(-max_integral_);
        Eigen::VectorXd integral = ki_.array() * integral_.array();

        Eigen::VectorXd derivative = kd_.array() * ((error - previous_error_).array() / dt_);
        derivative = derivative.cwiseMin(max_derivative_).cwiseMax(-max_derivative_);

        previous_error_ = error;

        // Compute torques
        Eigen::VectorXd torques = proportional + integral + derivative + feedforward;

        // Clamp torques
        for (int i = 0; i < torques.size(); ++i) {
            torques[i] = std::clamp(torques[i], -max_torque_, max_torque_);
        }

        // Publish torques
        auto torque_msg = sensor_msgs::msg::JointState();
        torque_msg.header.stamp = now();
        torque_msg.effort.resize(torques.size());
        Eigen::VectorXd::Map(&torque_msg.effort[0], torques.size()) = torques;

        // Optionally log command torques
        //Eigen::IOFormat TorqueFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "[", "]");
        //std::stringstream torque_stream;
        //torque_stream << "Publishing torques: [" << torques.format(TorqueFormat) << "]";
        //RCLCPP_INFO(this->get_logger(), "%s", torque_stream.str().c_str());

        torque_publisher_->publish(torque_msg);

    } else {
        RCLCPP_WARN(this->get_logger(), "Received joint state with mismatched size");
    }
}

void PIDControllerNode::setDesiredPositionsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Handle setting desired positions
    if (msg->position.size() == static_cast<size_t>(model_.nq)) {
        q_desired_ = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());

        // Optionally log desired joint positions
        //Eigen::IOFormat PositionFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "[", "]");
        //std::stringstream position_stream;
        //position_stream << "Setting desired positions: [" << q_desired_.format(PositionFormat) << "]";
        //RCLCPP_INFO(this->get_logger(), "%s", position_stream.str().c_str());

    } else {
        RCLCPP_WARN(this->get_logger(), "Received desired positions with mismatched size");
    }
}

void PIDControllerNode::update() {
    // Placeholder update function, everything happens in callbacks for now
}

rcl_interfaces::msg::SetParametersResult PIDControllerNode::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    std::vector<double> kp = kp_.size() ? std::vector<double>(kp_.data(), kp_.data() + kp_.size()) : std::vector<double>();
    std::vector<double> ki = ki_.size() ? std::vector<double>(ki_.data(), ki_.data() + ki_.size()) : std::vector<double>();
    std::vector<double> kd = kd_.size() ? std::vector<double>(kd_.data(), kd_.data() + kd_.size()) : std::vector<double>();
    bool gains_changed = false;

    for (const auto& param : parameters) {
        if (param.get_name() == "kp") {
            kp = param.as_double_array();
            gains_changed = true;
        } else if (param.get_name() == "ki") {
            ki = param.as_double_array();
            gains_changed = true;
        } else if (param.get_name() == "kd") {
            kd = param.as_double_array();
            gains_changed = true;
        } else if (param.get_name() == "use_gravity_ff") {
            use_gravity_ff_ = param.as_bool();
        } else if (param.get_name() == "use_coriolis_ff") {
            use_coriolis_ff_ = param.as_bool();
        } else if (param.get_name() == "reset_integral_on_gain_change") {
            reset_integral_on_gain_change_ = param.as_bool();
        }
    }

    if (gains_changed) {
        if (kp.size() != static_cast<size_t>(model_.nq) ||
            ki.size() != static_cast<size_t>(model_.nq) ||
            kd.size() != static_cast<size_t>(model_.nq)) {
            result.successful = false;
            result.reason = "kp/ki/kd size must match model nq";
            return result;
        }

        kp_ = Eigen::VectorXd::Map(kp.data(), kp.size());
        ki_ = Eigen::VectorXd::Map(ki.data(), ki.size());
        kd_ = Eigen::VectorXd::Map(kd.data(), kd.size());

        if (reset_integral_on_gain_change_) {
            integral_.setZero();
            previous_error_.setZero();
        }
    }

    return result;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
