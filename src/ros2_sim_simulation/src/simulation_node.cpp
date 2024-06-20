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
#include <random>    // For random number generation

class SimulationNode;

class PIDController {
public:
    struct PIDGains {
        Eigen::VectorXd kp;
        Eigen::VectorXd ki;
        Eigen::VectorXd kd;
    };

    PIDController(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q_desired)
        : model_(model), data_(data), q_desired_(q_desired) {
        // Initialize gains for each joint
        // Set the optimized PID gains for each joint
        Eigen::VectorXd kp(6), ki(6), kd(6);
        kp << 1.7688, 6.01637, 7.5294, 7.18778, 5.87989, 4.0;
        ki << 1.57731, 6.34717, 6.3343, 6.17437, 7.02989, 0.0;
        kd << 2.57169, 7.9477, 5.98217, 5.26123, 1.61688, 0.0;
        setGains(kp, ki, kd);
    }

    void setGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& ki, const Eigen::VectorXd& kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setDesiredPositions(const Eigen::VectorXd& q_desired) {
        q_desired_ = q_desired;
    }

    Eigen::VectorXd computeTorques(const Eigen::VectorXd& q, const Eigen::VectorXd& v, double dt) {
        pinocchio::computeGeneralizedGravity(model_, data_, q);
        Eigen::VectorXd gravityCompensationTerms = data_.g;

        Eigen::VectorXd error = q_desired_ - q;
        Eigen::VectorXd proportional = kp_.array() * error.array();

        integral_ += error * dt;
        integral_ = integral_.cwiseMin(max_integral_).cwiseMax(-max_integral_);
        Eigen::VectorXd integral = ki_.array() * integral_.array();

        Eigen::VectorXd derivative = kd_.array() * ((error - previous_error_).array() / dt);
        derivative = derivative.cwiseMin(max_derivative_).cwiseMax(-max_derivative_);

        previous_error_ = error;

        Eigen::VectorXd torque = proportional + integral + derivative + gravityCompensationTerms;

        for (int i = 0; i < torque.size(); ++i) {
            torque[i] = std::clamp(torque[i], -max_torque_, max_torque_);
        }

        return torque;
    }

    void optimizeGains(SimulationNode& simulation_node);

private:
    const pinocchio::Model& model_;
    pinocchio::Data& data_;
    Eigen::VectorXd q_desired_;
    Eigen::VectorXd integral_ = Eigen::VectorXd::Zero(model_.nq);
    Eigen::VectorXd previous_error_ = Eigen::VectorXd::Zero(model_.nv);

    Eigen::VectorXd kp_;
    Eigen::VectorXd ki_;
    Eigen::VectorXd kd_;
    double max_torque_ = 100.0;

    Eigen::VectorXd max_integral_ = Eigen::VectorXd::Constant(model_.nq, 100.0);
    Eigen::VectorXd max_derivative_ = Eigen::VectorXd::Constant(model_.nq, 100.0);

    void initializeGains() {
        std::default_random_engine re;
        std::uniform_real_distribution<double> unif(0.0, 10.0);

        kp_ = Eigen::VectorXd::Zero(model_.nq);
        ki_ = Eigen::VectorXd::Zero(model_.nq);
        kd_ = Eigen::VectorXd::Zero(model_.nq);

        for (int i = 0; i < model_.nq; ++i) {
            kp_[i] = unif(re);
            ki_[i] = unif(re);
            kd_[i] = unif(re);
        }
    }

    double evaluateFitness(SimulationNode& simulation_node, const PIDGains& gains);
    std::vector<PIDGains> generateInitialPopulation(int population_size);
    PIDGains crossover(const PIDGains& parent1, const PIDGains& parent2);
    void mutate(PIDGains& individual);
};

class SimulationNode : public rclcpp::Node {
public:
    SimulationNode() : Node("ros2_sim_simulation_node") {
        declare_parameter("urdf_path", "");
        get_parameter("urdf_path", urdf_path);

        dt = 0.001;
        damping_factor = -0.01;

        error_metric_ = 0.0;
        simulation_steps_ = 0;

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        q_ = Eigen::VectorXd::Zero(model_.nq);
        v_ = Eigen::VectorXd::Zero(model_.nv);
        a_ = Eigen::VectorXd::Zero(model_.nv);
        tau_ = Eigen::VectorXd::Zero(model_.nv);

        joint_names_.reserve(model_.njoints - 1);
        for (size_t i = 1; i < model_.names.size(); ++i) {
            if (model_.names[i].find("joint") != std::string::npos) {
                joint_names_.push_back(model_.names[i]);
            }
        }

        q_desired_ = Eigen::VectorXd::Zero(model_.nq);
        pid_controller_ = std::make_unique<PIDController>(model_, data_, q_desired_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt * 1000)),
            std::bind(&SimulationNode::update, this)
        );
    }

    void reset() {
        q_ = Eigen::VectorXd::Zero(model_.nq);
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

        tau_ = damping_factor * v_;
        tau_ += pid_controller_->computeTorques(q_, v_, dt);
        a_ = pinocchio::aba(model_, data_, q_, v_, tau_);
        v_ += a_ * dt;
        q_ = pinocchio::integrate(model_, q_, v_ * dt);

        for (int i = 0; i < q_.size(); ++i) {
            q_[i] = std::clamp(q_[i], model_.lowerPositionLimit[i], model_.upperPositionLimit[i]);
        }

        if (q_.hasNaN() || v_.hasNaN() || a_.hasNaN() || tau_.hasNaN()) {
            RCLCPP_WARN(this->get_logger(), "NaN detected in joint state. Resetting simulation.");
            reset();
            return;
        }

        Eigen::VectorXd error = q_desired_ - q_;
        error_metric_ += error.squaredNorm();
        simulation_steps_++;

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

    std::unique_ptr<PIDController> pid_controller_;
    Eigen::VectorXd q_desired_;

    double max_acceleration_ = 10.0;

    friend class PIDController;
};

double PIDController::evaluateFitness(SimulationNode& simulation_node, const PIDGains& gains) {
    setGains(gains.kp, gains.ki, gains.kd);
    simulation_node.reset();

    for (int i = 0; i < 1200; ++i) {
        simulation_node.update();
    }

    return simulation_node.getErrorMetric();
}

std::vector<PIDController::PIDGains> PIDController::generateInitialPopulation(int population_size) {
    std::vector<PIDGains> population;
    std::default_random_engine re;
    std::uniform_real_distribution<double> unif(0.0, 10.0);

    for (int i = 0; i < population_size; ++i) {
        PIDGains gains;
        gains.kp = Eigen::VectorXd::Zero(model_.nq);
        gains.ki = Eigen::VectorXd::Zero(model_.nq);
        gains.kd = Eigen::VectorXd::Zero(model_.nq);

        for (int j = 0; j < model_.nq; ++j) {
            gains.kp[j] = unif(re);
            gains.ki[j] = unif(re);
            gains.kd[j] = unif(re);
        }

        population.push_back(gains);
    }

    return population;
}

PIDController::PIDGains PIDController::crossover(const PIDGains& parent1, const PIDGains& parent2) {
    PIDGains child;
    child.kp = (parent1.kp + parent2.kp) / 2.0;
    child.ki = (parent1.ki + parent2.ki) / 2.0;
    child.kd = (parent1.kd + parent2.kd) / 2.0;

    return child;
}

void PIDController::mutate(PIDGains& individual) {
    std::default_random_engine re;
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    if (dis(re) < 0.1) {
        std::uniform_real_distribution<double> mutation_dis(-1.0, 1.0);

        for (int i = 0; i < model_.nq; ++i) {
            individual.kp[i] += mutation_dis(re);
            individual.ki[i] += mutation_dis(re);
            individual.kd[i] += mutation_dis(re);

            individual.kp[i] = std::max(0.0, std::min(10.0, individual.kp[i]));
            individual.ki[i] = std::max(0.0, std::min(10.0, individual.ki[i]));
            individual.kd[i] = std::max(0.0, std::min(10.0, individual.kd[i]));
        }
    }
}

void PIDController::optimizeGains(SimulationNode& simulation_node) {
    const int population_size = 20;
    const int generations = 50;
    const double elite_fraction = 0.2;

    initializeGains();

    auto population = generateInitialPopulation(population_size);
    std::vector<double> fitness(population_size);

    for (int generation = 0; generation < generations; ++generation) {
        for (int i = 0; i < population_size; ++i) {
            fitness[i] = evaluateFitness(simulation_node, population[i]);
        }

        std::vector<std::pair<double, PIDGains>> fitness_population;
        for (int i = 0; i < population_size; ++i) {
            fitness_population.push_back({fitness[i], population[i]});
        }

        std::sort(fitness_population.begin(), fitness_population.end(),
                  [](const std::pair<double, PIDGains>& a, const std::pair<double, PIDGains>& b) {
                      return a.first < b.first;
                  });

        int elite_count = static_cast<int>(elite_fraction * population_size);
        std::vector<PIDGains> new_population;
        for (int i = 0; i < elite_count; ++i) {
            new_population.push_back(fitness_population[i].second);
        }

        std::default_random_engine re;
        std::uniform_int_distribution<int> dis(0, elite_count - 1);

        while (new_population.size() < population_size) {
            PIDGains parent1 = fitness_population[dis(re)].second;
            PIDGains parent2 = fitness_population[dis(re)].second;
            PIDGains child = crossover(parent1, parent2);
            mutate(child);
            new_population.push_back(child);
        }

        population = new_population;

        // Logging gains for the best individual of the current generation
        PIDGains best_gains = fitness_population[0].second;
        std::stringstream kp_stream, ki_stream, kd_stream;
        kp_stream << best_gains.kp.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));
        ki_stream << best_gains.ki.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));
        kd_stream << best_gains.kd.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));

        RCLCPP_INFO(simulation_node.get_logger(), "Generation %d: Best fitness = %f, Best gains: Kp=%s, Ki=%s, Kd=%s", generation, fitness_population[0].first, kp_stream.str().c_str(), ki_stream.str().c_str(), kd_stream.str().c_str());
    }

    // Final best gains after optimization
    PIDGains best_gains = population[0];
    setGains(best_gains.kp, best_gains.ki, best_gains.kd);

    std::stringstream kp_stream, ki_stream, kd_stream;
    kp_stream << best_gains.kp.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));
    ki_stream << best_gains.ki.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));
    kd_stream << best_gains.kd.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));

    RCLCPP_INFO(simulation_node.get_logger(), "Optimized PID gains: Kp=%s, Ki=%s, Kd=%s", kp_stream.str().c_str(), ki_stream.str().c_str(), kd_stream.str().c_str());
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulationNode>();

    Eigen::VectorXd q_desired = Eigen::VectorXd::Constant(6, 0.2);
    node->getPIDController()->setDesiredPositions(q_desired);
    //node->getPIDController()->optimizeGains(*node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
