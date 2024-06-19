#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm> // For std::clamp

class SimulationNode;

class PIDController {
public:
    PIDController(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q_desired)
        : model_(model), data_(data), q_desired_(q_desired), gravity_(Eigen::VectorXd::Zero(model.nv))
    {
        pinocchio::computeGeneralizedGravity(model_, data_, q_desired_);
    }

    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    Eigen::VectorXd computeTorques(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a, double dt) {
        // Proportional term
        Eigen::VectorXd error = q_desired_ - q;
        Eigen::VectorXd proportional = kp_ * error;

        // Integral term
        integral_ += error * dt;
        Eigen::VectorXd integral = ki_ * integral_;

        // Derivative term
        Eigen::VectorXd derivative = kd_ * (error - previous_error_) / dt;

        // Update previous error
        previous_error_ = error;

        // Total torque
        Eigen::VectorXd torque = proportional + integral + derivative;

        // Clamp the torque to prevent large values
        for (int i = 0; i < torque.size(); ++i) {
            torque[i] = std::clamp(torque[i], -max_torque_, max_torque_);
        }

        // Add gravity compensation
        torque -= gravity_;

        return torque;
    }

    void optimizeGains(SimulationNode& simulation_node);

private:
    const pinocchio::Model& model_;
    pinocchio::Data& data_;
    Eigen::VectorXd q_desired_;
    Eigen::VectorXd gravity_;
    Eigen::VectorXd integral_ = Eigen::VectorXd::Zero(model_.nq);
    Eigen::VectorXd previous_error_ = Eigen::VectorXd::Zero(model_.nv);

    double kp_ = 0.0;  // Proportional gain
    double ki_ = 0.5;  // Integral gain
    double kd_ = 17.0;  // Derivative gain
    double max_torque_ = 10.0; // Maximum allowable torque
};

class SimulationNode : public rclcpp::Node {
public:
    SimulationNode() : Node("ros2_sim_simulation_node") {
        // Declare parameters
        declare_parameter("urdf_path", "/home/ws/src/ros2_sim_ur3_description/urdf/robot.urdf");
        declare_parameter("damping_factor", 0.1);
        declare_parameter("dt", 0.01);

        error_metric_ = 0.0;
        simulation_steps_ = 0;

        // Get parameters
        urdf_path = get_parameter("urdf_path").as_string();
        damping_factor = get_parameter("damping_factor").as_double();
        dt = get_parameter("dt").as_double();

        RCLCPP_INFO(this->get_logger(), "URDF path: %s", urdf_path.c_str());
        RCLCPP_INFO(this->get_logger(), "damping factor: %f", damping_factor);
        RCLCPP_INFO(this->get_logger(), "Time step (dt): %f", dt);

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Load the URDF model
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        // Initialize joint states
        q_ = Eigen::VectorXd::Zero(model_.nv);
        v_ = Eigen::VectorXd::Zero(model_.nv);
        a_ = Eigen::VectorXd::Zero(model_.nv);
        tau_ = Eigen::VectorXd::Zero(model_.nv);

        // Extract joint names
        joint_names_.reserve(model_.njoints - 1);  // excluding the universe joint
        for (size_t i = 1; i < model_.names.size(); ++i) {
            if (model_.names[i].find("joint") != std::string::npos) {
                joint_names_.push_back(model_.names[i]);
            }
        }

        // Initialize PID controller with desired joint positions
        q_desired_ = Eigen::VectorXd::Zero(model_.nq);  // Initialize with zero position or a realistic initial position
        pid_controller_ = std::make_unique<PIDController>(model_, data_, q_desired_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt * 1000)),
            std::bind(&SimulationNode::update, this)
        );
    }

    void reset() {
        q_ = Eigen::VectorXd::Zero(model_.nv);
        v_ = Eigen::VectorXd::Zero(model_.nv);
        a_ = Eigen::VectorXd::Zero(model_.nv);
        tau_ = Eigen::VectorXd::Zero(model_.nv);
        error_metric_ = 0.0;
        simulation_steps_ = 0;
    }

    double getErrorMetric() const {
        return error_metric_ / simulation_steps_;
    }

    PIDController* getPIDController() const {
        return pid_controller_.get();
    }

private:
    void update() {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Apply damping
        tau_ = damping_factor * v_;

        // Compute torques from PID controller
        //tau_ += pid_controller_->computeTorques(q_, v_, Eigen::VectorXd::Zero(model_.nv), dt);

        // Compute forward dynamics
        a_ = pinocchio::aba(model_, data_, q_, v_, tau_);

        // Integrate to get new velocity and position
        v_ += a_ * dt;
        q_ = pinocchio::integrate(model_, q_, v_ * dt);

        // Clamp joint positions within limits
        for (int i = 0; i < q_.size(); ++i) {
            q_[i] = std::clamp(q_[i], model_.lowerPositionLimit[i], model_.upperPositionLimit[i]);
        }

        // Calculate error metric (MSE)
        Eigen::VectorXd error = q_desired_ - q_;
        error_metric_ += error.squaredNorm();
        simulation_steps_++;

        // Publish joint states
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = joint_names_;
        joint_state.position = std::vector<double>(q_.data(), q_.data() + q_.size());
        joint_state.velocity = std::vector<double>(v_.data(), v_.data() + v_.size());
        joint_state.effort = std::vector<double>(tau_.data(), tau_.data() + tau_.size());

        publisher_->publish(joint_state);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        auto sleep_time = std::chrono::milliseconds(static_cast<int>(dt * 1000)) - std::chrono::milliseconds(elapsed_time);

        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    std::string urdf_path;

    double error_metric_;
    int simulation_steps_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    pinocchio::Model model_;
    pinocchio::Data data_;
    std::vector<std::string> joint_names_;

    Eigen::VectorXd q_, v_, a_, tau_;
    double dt;
    double damping_factor;

    // PID controller
    std::unique_ptr<PIDController> pid_controller_;
    Eigen::VectorXd q_desired_;

    // Maximum allowable acceleration
    double max_acceleration_ = 10.0; // Adjust based on your robot's capabilities

    friend class PIDController;
};

void PIDController::optimizeGains(SimulationNode& simulation_node) {
    double best_error = std::numeric_limits<double>::max();
    double best_kp = kp_, best_ki = ki_, best_kd = kd_;
    int iter = 0;

    for (double kp = 0.0; kp < 20.0; kp += 0.5) {
        for (double ki = 0.0; ki < 20.0; ki += 0.5) {
            for (double kd = 15.0; kd < 20.0; kd += 0.5) {
                RCLCPP_INFO(simulation_node.get_logger(), "Iteration: %d / 8000\n\tTesting PID gains: Kp=%f, Ki=%f, Kd=%f", iter++, kp, ki, kd);
                setGains(kp, ki, kd);
                simulation_node.reset();  // Reset the simulation state

                // Run the simulation for a predefined number of steps
                for (int i = 0; i < 800; ++i) {
                    simulation_node.update();
                }

                double error = simulation_node.getErrorMetric();
                if (error < best_error) {
                    best_error = error;
                    best_kp = kp;
                    best_ki = ki;
                    best_kd = kd;
                }
                RCLCPP_INFO(simulation_node.get_logger(), "Current best PID gains: Kp=%f, Ki=%f, Kd=%f with error=%f", best_kp, best_ki, best_kd, best_error);
            }
        }
    }

    RCLCPP_INFO(simulation_node.get_logger(), "Finished testing gains. Best PID gains: Kp=%f, Ki=%f, Kd=%f with error=%f", best_kp, best_ki, best_kd, best_error);
    setGains(best_kp, best_ki, best_kd);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulationNode>();

    // Run if you want to optimize gains
    //node->getPIDController()->optimizeGains(*node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
