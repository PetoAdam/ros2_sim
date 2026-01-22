#!/usr/bin/env python3

"""Pick-and-place demo using the ros2_sim custom motion planner action.

Runs a simple sequence:
  home -> pre-pick -> pick -> lift -> pre-place -> place -> retreat -> home

This intentionally has small waits at key positions so it's visually obvious.

Requirements:
- `ros2_sim_motion_planner` launched (it provides the `plan_and_execute` action server)
  e.g. `ros2 launch ros2_sim_motion_planner motion_planner.launch.py`
  or your workspace's `launch_all_with_custom_planner.launch.py`.
"""

from __future__ import annotations

import sys
import time
import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from ros2_sim_msgs.action import PlanAndExecute


@dataclass(frozen=True)
class PlannerDefaults:
    planning_pipeline: str = "pilz_industrial_motion_planner"
    lin_planner_id: str = "LIN"
    ptp_planner_id: str = "PTP"
    timeout_sec: float = 8.0
    max_velocity_scaling_factor: float = 0.20
    max_acceleration_scaling_factor: float = 0.15


def _sleep_with_spin(node: Node, seconds: float) -> None:
    end = time.time() + seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def _pose_xyz_identity(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    return pose


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def _pose_xyz_tool_down(x: float, y: float, z: float, yaw_rad: float = 0.0) -> Pose:
    """Pose with tool pointing down (flip about X) + configurable yaw."""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)

    # roll=pi flips tool Z to point down; yaw controls in-plane heading.
    qx, qy, qz, qw = _quat_from_rpy(math.pi, 0.0, float(yaw_rad))
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class PickAndPlaceDemo(Node):
    def __init__(self) -> None:
        super().__init__("pick_and_place_demo")
        self._client = ActionClient(self, PlanAndExecute, "plan_and_execute")
        self._defaults = PlannerDefaults()

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        self.get_logger().info("Waiting for action server: plan_and_execute")
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def _make_common_goal_fields(self, goal: PlanAndExecute.Goal) -> None:
        goal.timeout = float(self._defaults.timeout_sec)
        goal.max_velocity_scaling_factor = float(self._defaults.max_velocity_scaling_factor)
        goal.max_acceleration_scaling_factor = float(self._defaults.max_acceleration_scaling_factor)

    def _send_cartesian_goal(self, pose: Pose, planner_id: str, vel: float, acc: float) -> bool:
        goal = PlanAndExecute.Goal()
        goal.target_type = PlanAndExecute.Goal.CARTESIAN
        goal.target_pose = pose
        goal.planning_pipeline = self._defaults.planning_pipeline
        goal.planner_id = planner_id
        goal.timeout = float(self._defaults.timeout_sec)
        goal.max_velocity_scaling_factor = float(vel)
        goal.max_acceleration_scaling_factor = float(acc)

        self.get_logger().info(
            f"CARTESIAN {planner_id} to [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] "
            f"(vel={vel:.2f} acc={acc:.2f})"
        )
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f"Failed: {result.message}")
        return bool(result.success)

    def send_lin_with_retries(self, pose: Pose) -> bool:
        """Send a LIN goal, retrying only by lowering vel/acc scaling.

        This intentionally never falls back to PTP.
        """

        candidates = [
            (self._defaults.max_velocity_scaling_factor, self._defaults.max_acceleration_scaling_factor),
            (0.15, 0.12),
            (0.10, 0.08),
        ]
        for vel, acc in candidates:
            ok = self._send_cartesian_goal(pose, self._defaults.lin_planner_id, vel=vel, acc=acc)
            if ok:
                return True
            self.get_logger().warn("LIN failed; retrying with lower limits...")
            _sleep_with_spin(self, 0.15)
        return False

    def send_ptp_with_retries(self, pose: Pose) -> bool:
        """Send a PTP goal (allowed for the carry-rotation), retrying by lowering limits."""
        candidates = [
            (self._defaults.max_velocity_scaling_factor, self._defaults.max_acceleration_scaling_factor),
            (0.20, 0.15),
            (0.15, 0.12),
            (0.10, 0.08),
        ]
        for vel, acc in candidates:
            ok = self._send_cartesian_goal(pose, self._defaults.ptp_planner_id, vel=vel, acc=acc)
            if ok:
                return True
            self.get_logger().warn("PTP failed; retrying with lower limits...")
            _sleep_with_spin(self, 0.15)
        return False

    def run(self) -> int:
        if not self.wait_for_server(timeout_sec=8.0):
            self.get_logger().error(
                "Action server not available. Start the motion planner launch first."
            )
            return 2

        # Tool-down poses (end-effector points down, not up).
        pre_pick = _pose_xyz_tool_down(0.35, 0.15, 0.30, yaw_rad=0.0)
        pick = _pose_xyz_tool_down(0.35, 0.15, 0.20, yaw_rad=0.0)

        # Place target is "behind" the robot: closer-in X and opposite Y.
        # No extra 180° rotation stage (keeps the demo snappy).
        pre_place = _pose_xyz_tool_down(0.25, -0.20, 0.32, yaw_rad=0.0)
        place = _pose_xyz_tool_down(0.25, -0.20, 0.22, yaw_rad=0.0)

        self.get_logger().info("=== PICK & PLACE DEMO: start ===")

        # Always start with a PTP move to the starting pose.
        if not self.send_ptp_with_retries(pre_pick):
            return 1
        _sleep_with_spin(self, 0.4)

        # Descend vertically (LIN)
        if not self.send_lin_with_retries(pick):
            return 1
        self.get_logger().info("At PICK pose: waiting (simulate grasp)")
        _sleep_with_spin(self, 0.8)

        # Lift back up
        # Lift vertically (LIN)
        if not self.send_lin_with_retries(pre_pick):
            return 1
        _sleep_with_spin(self, 0.4)

        if not self.send_ptp_with_retries(pre_place):
            return 1
        _sleep_with_spin(self, 0.4)

        # Descend vertically (LIN)
        if not self.send_lin_with_retries(place):
            return 1
        self.get_logger().info("At PLACE pose: waiting (simulate release)")
        _sleep_with_spin(self, 0.8)

        # Retreat
        if not self.send_lin_with_retries(pre_place):
            return 1
        _sleep_with_spin(self, 0.4)

        self.get_logger().info("=== PICK & PLACE DEMO: done ===")
        return 0


def main(argv: Optional[List[str]] = None) -> None:
    rclpy.init(args=argv)
    node = PickAndPlaceDemo()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main(sys.argv)
