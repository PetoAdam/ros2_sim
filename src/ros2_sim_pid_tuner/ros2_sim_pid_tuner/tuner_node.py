import threading
import random
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

try:
    from skopt import gp_minimize
    from skopt.space import Real
except Exception:
    gp_minimize = None
    Real = None


class PidTunerNode(Node):
    def __init__(self) -> None:
        super().__init__("ros2_sim_pid_tuner")

        self.declare_parameter("pid_node_name", "ros2_sim_pid_controller_node")
        self.declare_parameter("reset_service", "/reset_simulation")
        self.declare_parameter("joint_state_topic", "/sim_joint_states")
        self.declare_parameter("desired_positions_topic", "/desired_positions")
        self.declare_parameter("eval_time", 2.0)
        self.declare_parameter("warmup_time", 0.5)
        self.declare_parameter("log_every_n_samples", 200)

        self.declare_parameter("use_current_gains", True)
        self.declare_parameter("base_kp", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("base_ki", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("base_kd", rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.declare_parameter("kp_multipliers", [0.5, 1.0, 1.5])
        self.declare_parameter("ki_multipliers", [0.0, 0.5, 1.0])
        self.declare_parameter("kd_multipliers", [0.5, 1.0, 1.5])
        self.declare_parameter("max_trials", 50)
        self.declare_parameter("tuning_algorithm", "grid")
        self.declare_parameter("bayes_calls", 30)
        self.declare_parameter("tuning_mode", "combined")
        self.declare_parameter("per_joint_indices", rclpy.Parameter.Type.INTEGER_ARRAY)

        self.declare_parameter("desired_positions", [0.2, 0.2, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("desired_positions_sets", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("desired_positions_set_size", 0)
        self.declare_parameter("randomize_sets", False)
        self.declare_parameter("set_seed", 42)
        self.declare_parameter("use_gravity_ff", True)
        self.declare_parameter("use_coriolis_ff", False)
        self.declare_parameter("output_yaml", "/home/ws/src/ros2_sim_pid_controller/config/pid_controller_config.tuned.yaml")
        self.declare_parameter("weight_abs_error", 1.0)
        self.declare_parameter("weight_velocity", 0.05)
        self.declare_parameter("weight_final_error", 0.5)
        self.declare_parameter("weight_overshoot", 0.5)
        self.declare_parameter("max_overshoot_ratio", 0.3)
        self.declare_parameter("overshoot_penalty", 5.0)
        self.declare_parameter("log_full_gains", False)

        self.pid_node_name = self.get_parameter("pid_node_name").get_parameter_value().string_value
        self.reset_service_name = self.get_parameter("reset_service").get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        self.desired_positions_topic = self.get_parameter("desired_positions_topic").get_parameter_value().string_value
        self.eval_time = float(self.get_parameter("eval_time").value)
        self.warmup_time = float(self.get_parameter("warmup_time").value)
        self.log_every_n_samples = int(self.get_parameter("log_every_n_samples").value)

        self.use_current_gains = bool(self.get_parameter("use_current_gains").value)
        self.base_kp = list(self.get_parameter("base_kp").value)
        self.base_ki = list(self.get_parameter("base_ki").value)
        self.base_kd = list(self.get_parameter("base_kd").value)

        self.kp_multipliers = list(self.get_parameter("kp_multipliers").value)
        self.ki_multipliers = list(self.get_parameter("ki_multipliers").value)
        self.kd_multipliers = list(self.get_parameter("kd_multipliers").value)
        self.max_trials = int(self.get_parameter("max_trials").value)
        self.tuning_algorithm = str(self.get_parameter("tuning_algorithm").value)
        self.bayes_calls = int(self.get_parameter("bayes_calls").value)
        self.tuning_mode = str(self.get_parameter("tuning_mode").value)
        per_joint_param = self.get_parameter_or("per_joint_indices", [])
        per_joint_value = per_joint_param.value if hasattr(per_joint_param, "value") else per_joint_param
        self.per_joint_indices = list(per_joint_value)

        self.desired_positions = list(self.get_parameter("desired_positions").value)
        self.desired_positions_sets = list(self.get_parameter_or("desired_positions_sets", []).value)
        self.desired_positions_set_size = int(self.get_parameter("desired_positions_set_size").value)
        self.randomize_sets = bool(self.get_parameter("randomize_sets").value)
        self.set_seed = int(self.get_parameter("set_seed").value)
        self.use_gravity_ff = bool(self.get_parameter("use_gravity_ff").value)
        self.use_coriolis_ff = bool(self.get_parameter("use_coriolis_ff").value)
        self.output_yaml = self.get_parameter("output_yaml").get_parameter_value().string_value
        self.weight_abs_error = float(self.get_parameter("weight_abs_error").value)
        self.weight_velocity = float(self.get_parameter("weight_velocity").value)
        self.weight_final_error = float(self.get_parameter("weight_final_error").value)
        self.weight_overshoot = float(self.get_parameter("weight_overshoot").value)
        self.max_overshoot_ratio = float(self.get_parameter("max_overshoot_ratio").value)
        self.overshoot_penalty = float(self.get_parameter("overshoot_penalty").value)
        self.log_full_gains = bool(self.get_parameter("log_full_gains").value)

        self.param_client = AsyncParameterClient(self, self.pid_node_name)
        self.reset_client = self.create_client(Empty, self.reset_service_name)

        self.desired_pub = self.create_publisher(JointState, self.desired_positions_topic, 10)
        self.joint_state_sub = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 50)

        self.collecting = False
        self.iae_sum = 0.0
        self.vel_sum = 0.0
        self.sample_count = 0
        self.vel_count = 0
        self.samples_since_log = 0
        self.latest_positions = None
        self.latest_velocities = None
        self.initial_error = None
        self.max_overshoot = None
        self.pid_dt = 0.005
        self.error_indices = None
        self.position_sets = self._build_position_sets()

        self.started = False
        self.start_timer = self.create_timer(0.5, self.start_once)

        self.get_logger().info("PID tuner node initialized.")
        self.get_logger().info(f"PID node: {self.pid_node_name}")
        self.get_logger().info(f"Reset service: {self.reset_service_name}")
        self.get_logger().info(f"Joint state topic: {self.joint_state_topic}")
        self.get_logger().info(f"Desired positions topic: {self.desired_positions_topic}")
        self.get_logger().info(f"Tuning mode: {self.tuning_mode}")
        self.get_logger().info(f"Tuning algorithm: {self.tuning_algorithm}")
        if self.position_sets:
            self.get_logger().info(f"Using {len(self.position_sets)} desired position sets.")
        else:
            self.get_logger().info("Using single desired position set.")

    def start_once(self) -> None:
        if self.started:
            return
        self.started = True
        self.start_timer.cancel()
        worker = threading.Thread(target=self.run_tuning, daemon=True)
        worker.start()

    def _build_position_sets(self) -> List[List[float]]:
        if not self.desired_positions_sets or self.desired_positions_set_size <= 0:
            return []
        set_size = self.desired_positions_set_size
        if set_size != len(self.desired_positions):
            self.get_logger().warn(
                "desired_positions_set_size does not match desired_positions length; ignoring sets."
            )
            return []
        flat = list(self.desired_positions_sets)
        if len(flat) % set_size != 0:
            self.get_logger().warn(
                "desired_positions_sets length is not divisible by desired_positions_set_size; ignoring sets."
            )
            return []
        sets = [flat[i : i + set_size] for i in range(0, len(flat), set_size)]
        if not sets:
            return []
        return sets

    def prepare_single_set(self) -> None:
        if not self.position_sets:
            self.publish_desired_positions()
            time.sleep(self.warmup_time)

    def joint_state_callback(self, msg: JointState) -> None:
        if len(msg.position) != len(self.desired_positions):
            return
        self.latest_positions = list(msg.position)
        if msg.velocity:
            self.latest_velocities = list(msg.velocity)
        if not self.collecting:
            return

        error = [self.desired_positions[i] - msg.position[i] for i in range(len(self.desired_positions))]
        if self.initial_error is None:
            self.initial_error = list(error)
            self.max_overshoot = [0.0 for _ in error]

        if self.initial_error is not None and self.max_overshoot is not None:
            for i, err in enumerate(error):
                initial = self.initial_error[i]
                if initial == 0.0:
                    continue
                if (initial > 0 and err < 0) or (initial < 0 and err > 0):
                    self.max_overshoot[i] = max(self.max_overshoot[i], abs(err))
        if self.error_indices:
            selected = [error[i] for i in self.error_indices]
            self.iae_sum += sum(abs(e) for e in selected)
            self.sample_count += len(selected)
            if self.latest_velocities and len(self.latest_velocities) == len(self.desired_positions):
                vel_selected = [self.latest_velocities[i] for i in self.error_indices]
                self.vel_sum += sum(abs(v) for v in vel_selected)
                self.vel_count += len(vel_selected)
        else:
            self.iae_sum += sum(abs(e) for e in error)
            self.sample_count += len(error)
            if self.latest_velocities and len(self.latest_velocities) == len(self.desired_positions):
                self.vel_sum += sum(abs(v) for v in self.latest_velocities)
                self.vel_count += len(self.latest_velocities)
        self.samples_since_log += 1

        if self.samples_since_log >= self.log_every_n_samples:
            self.samples_since_log = 0
            mean_abs = self.iae_sum / max(self.sample_count, 1)
            self.get_logger().info(f"Collecting... mean_abs_error={mean_abs:.6f}")

    def run_tuning(self) -> None:
        self.get_logger().info("Waiting for PID parameter service...")
        if not self.param_client.wait_for_services(timeout_sec=10.0):
            self.get_logger().error("PID parameter service not available.")
            return

        self.get_logger().info("Waiting for reset service...")
        if not self.reset_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Reset service not available.")
            return

        if self.use_current_gains:
            self.get_logger().info("Reading current gains from PID node.")
            if not self.fetch_current_gains():
                self.get_logger().warn("Failed to fetch gains, falling back to base_* params.")

        if not (self.base_kp and self.base_ki and self.base_kd):
            self.get_logger().error("Base gains are empty. Provide base_kp/base_ki/base_kd or enable use_current_gains.")
            return

        mode = self.tuning_mode.lower()
        if mode not in {"global", "per_joint", "combined"}:
            self.get_logger().error(f"Unknown tuning_mode '{self.tuning_mode}'. Use global, per_joint, or combined.")
            return

        current_kp = list(self.base_kp)
        current_ki = list(self.base_ki)
        current_kd = list(self.base_kd)

        if mode in {"global", "combined"}:
            self.get_logger().info("Starting global tuning (all joints scaled equally).")
            global_result = self.run_global_tuning(current_kp, current_ki, current_kd)
            if global_result is not None:
                current_kp, current_ki, current_kd, global_score = global_result
                self.get_logger().info(f"Global tuning best score={global_score:.6f}")
            else:
                self.get_logger().warn("Global tuning produced no valid result.")

        if mode in {"per_joint", "combined"}:
            self.get_logger().info("Starting per-joint tuning.")
            per_joint_result = self.run_per_joint_tuning(current_kp, current_ki, current_kd)
            if per_joint_result is not None:
                current_kp, current_ki, current_kd = per_joint_result
            else:
                self.get_logger().warn("Per-joint tuning produced no valid result.")

        self.get_logger().info("Tuning complete. Applying best gains.")
        if self.apply_gains(current_kp, current_ki, current_kd):
            self.get_logger().info(
                f"Final gains sample: kp[0]={current_kp[0]:.3f}, ki[0]={current_ki[0]:.3f}, kd[0]={current_kd[0]:.3f}"
            )
            self.write_tuned_yaml(current_kp, current_ki, current_kd)
        else:
            self.get_logger().warn("Failed to apply final gains.")

    def fetch_current_gains(self) -> bool:
        future = self.param_client.get_parameters(["kp", "ki", "kd", "dt"])
        result = self.wait_future(future, timeout=5.0)
        if result is None:
            return False
        params = result.values
        if len(params) != 4:
            return False
        self.base_kp = list(params[0].double_array_value)
        self.base_ki = list(params[1].double_array_value)
        self.base_kd = list(params[2].double_array_value)
        self.pid_dt = params[3].double_value or self.pid_dt
        self.get_logger().info("Fetched current gains from PID node.")
        return True

    def run_global_tuning(self, base_kp: List[float], base_ki: List[float], base_kd: List[float]):
        algo = self.tuning_algorithm.lower()
        if algo == "bayes" and gp_minimize is None:
            self.get_logger().warn("Bayesian optimizer unavailable; falling back to grid.")
            algo = "grid"

        if algo == "bayes":
            kp_min, kp_max = min(self.kp_multipliers), max(self.kp_multipliers)
            ki_min, ki_max = min(self.ki_multipliers), max(self.ki_multipliers)
            kd_min, kd_max = min(self.kd_multipliers), max(self.kd_multipliers)
            space = [Real(kp_min, kp_max), Real(ki_min, ki_max), Real(kd_min, kd_max)]

            def objective(x):
                kp_mul, ki_mul, kd_mul = x
                kp = [k * kp_mul for k in base_kp]
                ki = [k * ki_mul for k in base_ki]
                kd = [k * kd_mul for k in base_kd]

                self.get_logger().info(
                    f"Global bayes trial: kp_mul={kp_mul:.3f}, ki_mul={ki_mul:.3f}, kd_mul={kd_mul:.3f}"
                )

                if not self.apply_gains(kp, ki, kd):
                    return float("inf")
                if not self.reset_sim():
                    return float("inf")
                self.prepare_single_set()
                return self.evaluate()

            calls = max(5, min(self.bayes_calls, self.max_trials))
            result = gp_minimize(objective, space, n_calls=calls, random_state=self.set_seed)
            best_kp_mul, best_ki_mul, best_kd_mul = result.x
            best_kp = [k * best_kp_mul for k in base_kp]
            best_ki = [k * best_ki_mul for k in base_ki]
            best_kd = [k * best_kd_mul for k in base_kd]
            return best_kp, best_ki, best_kd, result.fun

        total_combinations = len(self.kp_multipliers) * len(self.ki_multipliers) * len(self.kd_multipliers)
        self.get_logger().info(
            f"Global tuning: {total_combinations} combinations (max_trials={self.max_trials})."
        )

        best_score = float("inf")
        best_gains = None
        trials = 0

        for kp_mul in self.kp_multipliers:
            for ki_mul in self.ki_multipliers:
                for kd_mul in self.kd_multipliers:
                    if trials >= self.max_trials:
                        break

                    kp = [k * kp_mul for k in base_kp]
                    ki = [k * ki_mul for k in base_ki]
                    kd = [k * kd_mul for k in base_kd]

                    self.get_logger().info(
                        f"Global trial {trials + 1}: kp_mul={kp_mul}, ki_mul={ki_mul}, kd_mul={kd_mul}"
                    )

                    if not self.apply_gains(kp, ki, kd):
                        self.get_logger().error("Failed to apply gains; skipping trial.")
                        trials += 1
                        continue

                    if not self.reset_sim():
                        self.get_logger().error("Failed to reset sim; skipping trial.")
                        trials += 1
                        continue

                    self.prepare_single_set()

                    score = self.evaluate()
                    self.get_logger().info(f"Global trial {trials + 1} score={score:.6f}")

                    if score < best_score:
                        best_score = score
                        best_gains = (kp, ki, kd)
                        self.get_logger().info(f"New best global score={best_score:.6f}")

                    trials += 1

                if trials >= self.max_trials:
                    break
            if trials >= self.max_trials:
                break

        if best_gains is None:
            return None
        kp, ki, kd = best_gains
        return kp, ki, kd, best_score

    def run_per_joint_tuning(self, base_kp: List[float], base_ki: List[float], base_kd: List[float]):
        joint_count = len(base_kp)
        indices = self.per_joint_indices if self.per_joint_indices else list(range(joint_count))
        indices = [int(i) for i in indices if 0 <= int(i) < joint_count]
        if not indices:
            self.get_logger().error("No valid per_joint_indices; skipping per-joint tuning.")
            return None

        tuned_kp = list(base_kp)
        tuned_ki = list(base_ki)
        tuned_kd = list(base_kd)

        for joint_index in indices:
            self.get_logger().info(f"Per-joint tuning for joint {joint_index}.")
            algo = self.tuning_algorithm.lower()
            if algo == "bayes" and gp_minimize is None:
                self.get_logger().warn("Bayesian optimizer unavailable; falling back to grid.")
                algo = "grid"

            best_score = float("inf")
            best_triplet = None
            trials = 0

            if algo == "bayes":
                kp_min, kp_max = min(self.kp_multipliers), max(self.kp_multipliers)
                ki_min, ki_max = min(self.ki_multipliers), max(self.ki_multipliers)
                kd_min, kd_max = min(self.kd_multipliers), max(self.kd_multipliers)
                space = [Real(kp_min, kp_max), Real(ki_min, ki_max), Real(kd_min, kd_max)]

                def objective(x):
                    kp_mul, ki_mul, kd_mul = x
                    kp = list(tuned_kp)
                    ki = list(tuned_ki)
                    kd = list(tuned_kd)
                    kp[joint_index] = base_kp[joint_index] * kp_mul
                    ki[joint_index] = base_ki[joint_index] * ki_mul
                    kd[joint_index] = base_kd[joint_index] * kd_mul

                    self.get_logger().info(
                        f"Joint {joint_index} bayes trial: kp_mul={kp_mul:.3f}, ki_mul={ki_mul:.3f}, kd_mul={kd_mul:.3f}"
                    )

                    if not self.apply_gains(kp, ki, kd, log_index=joint_index):
                        return float("inf")
                    if not self.reset_sim():
                        return float("inf")
                    self.prepare_single_set()
                    return self.evaluate(indices=[joint_index])

                calls = max(5, min(self.bayes_calls, self.max_trials))
                result = gp_minimize(objective, space, n_calls=calls, random_state=self.set_seed)
                best_kp_mul, best_ki_mul, best_kd_mul = result.x
                best_triplet = (
                    base_kp[joint_index] * best_kp_mul,
                    base_ki[joint_index] * best_ki_mul,
                    base_kd[joint_index] * best_kd_mul,
                )
                best_score = result.fun
            else:
                for kp_mul in self.kp_multipliers:
                    for ki_mul in self.ki_multipliers:
                        for kd_mul in self.kd_multipliers:
                            if trials >= self.max_trials:
                                break

                            kp = list(tuned_kp)
                            ki = list(tuned_ki)
                            kd = list(tuned_kd)
                            kp[joint_index] = base_kp[joint_index] * kp_mul
                            ki[joint_index] = base_ki[joint_index] * ki_mul
                            kd[joint_index] = base_kd[joint_index] * kd_mul

                            self.get_logger().info(
                                f"Joint {joint_index} trial {trials + 1}: kp_mul={kp_mul}, ki_mul={ki_mul}, kd_mul={kd_mul}"
                            )

                            if not self.apply_gains(kp, ki, kd, log_index=joint_index):
                                self.get_logger().error("Failed to apply gains; skipping trial.")
                                trials += 1
                                continue

                            if not self.reset_sim():
                                self.get_logger().error("Failed to reset sim; skipping trial.")
                                trials += 1
                                continue

                            self.prepare_single_set()

                            score = self.evaluate(indices=[joint_index])
                            self.get_logger().info(
                                f"Joint {joint_index} trial {trials + 1} score={score:.6f}"
                            )

                            if score < best_score:
                                best_score = score
                                best_triplet = (kp[joint_index], ki[joint_index], kd[joint_index])
                                self.get_logger().info(
                                    f"New best joint {joint_index} score={best_score:.6f}"
                                )

                            trials += 1

                        if trials >= self.max_trials:
                            break
                    if trials >= self.max_trials:
                        break

            if best_triplet is None:
                self.get_logger().warn(f"No valid trials for joint {joint_index}.")
                continue

            tuned_kp[joint_index], tuned_ki[joint_index], tuned_kd[joint_index] = best_triplet
            self.get_logger().info(
                f"Joint {joint_index} tuned gains: kp={best_triplet[0]:.3f}, ki={best_triplet[1]:.3f}, kd={best_triplet[2]:.3f}"
            )

        return tuned_kp, tuned_ki, tuned_kd

    def apply_gains(self, kp: List[float], ki: List[float], kd: List[float], log_index: int | None = None) -> bool:
        params = [
            Parameter("kp", Parameter.Type.DOUBLE_ARRAY, kp),
            Parameter("ki", Parameter.Type.DOUBLE_ARRAY, ki),
            Parameter("kd", Parameter.Type.DOUBLE_ARRAY, kd),
            Parameter("use_gravity_ff", Parameter.Type.BOOL, self.use_gravity_ff),
            Parameter("use_coriolis_ff", Parameter.Type.BOOL, self.use_coriolis_ff),
        ]
        future = self.param_client.set_parameters(params)
        result = self.wait_future(future, timeout=5.0)
        if result is None:
            return False

        results: List[SetParametersResult] = result.results
        if not all(r.successful for r in results):
            reasons = ", ".join(r.reason for r in results if not r.successful)
            self.get_logger().error(f"Gain update rejected: {reasons}")
            return False

        if self.log_full_gains:
            self.get_logger().info(f"Applied gains kp={kp}")
            self.get_logger().info(f"Applied gains ki={ki}")
            self.get_logger().info(f"Applied gains kd={kd}")
        elif log_index is not None and 0 <= log_index < len(kp):
            self.get_logger().info(
                f"Applied gains (joint {log_index}): kp={kp[log_index]:.3f}, ki={ki[log_index]:.3f}, kd={kd[log_index]:.3f}"
            )
        else:
            self.get_logger().info(
                f"Applied gains sample: kp[0]={kp[0]:.3f}, ki[0]={ki[0]:.3f}, kd[0]={kd[0]:.3f}"
            )
        return True

    def reset_sim(self) -> bool:
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        result = self.wait_future(future, timeout=5.0)
        if result is None:
            return False
        self.get_logger().info("Simulation reset complete.")
        return True

    def wait_future(self, future, timeout: float):
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def publish_desired_positions(self, positions: List[float] | None = None) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if positions is not None:
            self.desired_positions = list(positions)
        msg.position = list(positions if positions is not None else self.desired_positions)
        self.desired_pub.publish(msg)
        if positions is not None:
            self.get_logger().info(f"Published desired positions: {positions}")
        else:
            self.get_logger().info(f"Published desired positions: {self.desired_positions}")

    def evaluate(self, indices: List[int] | None = None) -> float:
        if not self.position_sets:
            iae, vel, final_err, overshoot = self.collect_metrics(indices)
            return self.score_with_penalties(iae, vel, final_err, overshoot)

        sets = list(self.position_sets)
        if self.randomize_sets:
            rng = random.Random(self.set_seed)
            rng.shuffle(sets)

        scores = []
        for set_positions in sets:
            self.publish_desired_positions(set_positions)
            time.sleep(self.warmup_time)
            iae, vel, final_err, overshoot = self.collect_metrics(indices)
            score = self.score_with_penalties(iae, vel, final_err, overshoot)
            self.get_logger().info(
                f"Set score={score:.6f} (iae={iae:.6f}, vel={vel:.6f}, final={final_err:.6f}, overshoot={overshoot:.6f})"
            )
            scores.append(score)

        return sum(scores) / max(len(scores), 1)

    def collect_metrics(self, indices: List[int] | None = None) -> tuple[float, float, float, float]:
        self.error_indices = indices
        self.collecting = True
        self.iae_sum = 0.0
        self.vel_sum = 0.0
        self.sample_count = 0
        self.vel_count = 0
        self.samples_since_log = 0
        self.initial_error = None
        self.max_overshoot = None

        start = time.time()
        while time.time() - start < self.eval_time:
            time.sleep(0.01)

        self.collecting = False
        self.error_indices = None

        if self.sample_count == 0:
            return float("inf"), float("inf"), float("inf"), float("inf")

        iae = self.iae_sum / self.sample_count
        vel = self.vel_sum / max(self.vel_count, 1)
        final_err = self.compute_final_error(indices)
        overshoot = self.compute_overshoot_ratio(indices)
        return iae, vel, final_err, overshoot

    def score_with_penalties(self, iae: float, vel: float, final_err: float, overshoot: float) -> float:
        score = self.weight_abs_error * iae + self.weight_velocity * vel + self.weight_final_error * final_err
        score += self.weight_overshoot * overshoot
        if overshoot > self.max_overshoot_ratio:
            score += self.overshoot_penalty * (overshoot - self.max_overshoot_ratio)
        return score

    def compute_final_error(self, indices: List[int] | None = None) -> float:
        if not self.latest_positions:
            return float("inf")
        error = [self.desired_positions[i] - self.latest_positions[i] for i in range(len(self.desired_positions))]
        if indices:
            selected = [abs(error[i]) for i in indices]
            return sum(selected) / max(len(selected), 1)
        return sum(abs(e) for e in error) / max(len(error), 1)

    def compute_overshoot_ratio(self, indices: List[int] | None = None) -> float:
        if not self.initial_error or not self.max_overshoot:
            return 0.0
        ratios = []
        for i, initial in enumerate(self.initial_error):
            denom = abs(initial) if abs(initial) > 1e-6 else 1.0
            ratios.append(self.max_overshoot[i] / denom)
        if indices:
            ratios = [ratios[i] for i in indices]
        return sum(ratios) / max(len(ratios), 1)

    def write_tuned_yaml(self, kp: List[float], ki: List[float], kd: List[float]) -> None:
        try:
            lines = [
                "ros2_sim_pid_controller_node:",
                "  ros__parameters:",
                f"    kp: {kp}",
                f"    ki: {ki}",
                f"    kd: {kd}",
                f"    dt: {self.pid_dt}",
                f"    use_gravity_ff: {str(self.use_gravity_ff).lower()}",
                f"    use_coriolis_ff: {str(self.use_coriolis_ff).lower()}",
                "    allow_gain_updates: true",
                "    reset_integral_on_gain_change: true",
            ]
            with open(self.output_yaml, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            self.get_logger().info(f"Wrote tuned gains to {self.output_yaml}")
        except Exception as exc:
            self.get_logger().error(f"Failed to write tuned yaml: {exc}")


def main() -> None:
    rclpy.init()
    node = PidTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
