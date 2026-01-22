#!/usr/bin/env python3

"""Welding-path demo using the ros2_sim custom motion planner action.

This script moves to a safe approach pose, then traces a simple rectangular "seam"
in the XY plane using LIN moves (Pilz Industrial Motion Planner).

Small waits are added at the start and at each corner so it's obvious it's a
"welding"-style motion.

Requirements:
- `ros2_sim_motion_planner` launched (it provides the `plan_and_execute` action server)
  e.g. `ros2 launch ros2_sim_motion_planner motion_planner.launch.py`
  or your workspace's `launch_all_with_custom_planner.launch.py`.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from ros2_sim_msgs.action import PlanAndExecute


@dataclass(frozen=True)
class PlannerDefaults:
    planning_pipeline: str = "pilz_industrial_motion_planner"
    lin_planner_id: str = "LIN"
    timeout_sec: float = 10.0
    max_velocity_scaling_factor: float = 0.15
    max_acceleration_scaling_factor: float = 0.12


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

    qx, qy, qz, qw = _quat_from_rpy(math.pi, 0.0, float(yaw_rad))
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class WeldingDemo(Node):
    def __init__(self) -> None:
        super().__init__("welding_demo")
        self._client = ActionClient(self, PlanAndExecute, "plan_and_execute")
        self._defaults = PlannerDefaults()

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        self.get_logger().info("Waiting for action server: plan_and_execute")
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def _make_common_goal_fields(self, goal: PlanAndExecute.Goal) -> None:
        goal.timeout = float(self._defaults.timeout_sec)
        goal.max_velocity_scaling_factor = float(self._defaults.max_velocity_scaling_factor)
        goal.max_acceleration_scaling_factor = float(self._defaults.max_acceleration_scaling_factor)

    def _send_cartesian(self, pose: Pose, planner_id: str, vel: float, acc: float) -> bool:
        goal = PlanAndExecute.Goal()
        goal.target_type = PlanAndExecute.Goal.CARTESIAN
        goal.target_pose = pose
        goal.planning_pipeline = self._defaults.planning_pipeline
        goal.planner_id = planner_id
        goal.timeout = float(self._defaults.timeout_sec)
        goal.max_velocity_scaling_factor = float(vel)
        goal.max_acceleration_scaling_factor = float(acc)

        self.get_logger().info(
            f"CARTESIAN {planner_id} -> [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] "
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
            (0.12, 0.10),
            (0.10, 0.08),
        ]
        for vel, acc in candidates:
            ok = self._send_cartesian(pose, self._defaults.lin_planner_id, vel=vel, acc=acc)
            if ok:
                return True
            self.get_logger().warn("LIN failed; retrying with lower limits...")
            _sleep_with_spin(self, 0.15)
        return False

    def send_ptp_with_retries(self, pose: Pose) -> bool:
        candidates = [
            (self._defaults.max_velocity_scaling_factor, self._defaults.max_acceleration_scaling_factor),
            (0.12, 0.10),
            (0.10, 0.08),
        ]
        for vel, acc in candidates:
            ok = self._send_cartesian(pose, "PTP", vel=vel, acc=acc)
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

        self.get_logger().info("=== WELDING DEMO: start ===")

        # Always start with a PTP move to a safe approach above the seam.
        approach = _pose_xyz_tool_down(0.35, 0.00, 0.32, yaw_rad=0.0)
        if not self.send_ptp_with_retries(approach):
            return 1
        _sleep_with_spin(self, 0.5)

        # Define a simple rectangular seam (closed loop) in XY at a fixed Z.
        z_seam = 0.22
        cx, cy = 0.35, 0.00
        dx, dy = 0.08, 0.06

        seam_points: Sequence[Tuple[float, float, float]] = [
            (cx - dx, cy - dy, z_seam),
            (cx + dx, cy - dy, z_seam),
            (cx + dx, cy + dy, z_seam),
            (cx - dx, cy + dy, z_seam),
            (cx - dx, cy - dy, z_seam),
        ]

        # Move to seam start
        start_pose = _pose_xyz_tool_down(*seam_points[0], yaw_rad=0.0)
        if not self.send_lin_with_retries(start_pose):
            return 1
        self.get_logger().info("At seam START: waiting (ignite arc)")
        _sleep_with_spin(self, 1.0)

        # Trace the seam with short dwells at corners.
        for i, (x, y, z) in enumerate(seam_points[1:], start=1):
            corner_pose = _pose_xyz_tool_down(x, y, z, yaw_rad=0.0)
            if not self.send_lin_with_retries(corner_pose):
                return 1
            if i < len(seam_points) - 1:
                self.get_logger().info(f"Corner {i}: brief dwell")
                _sleep_with_spin(self, 0.4)

        self.get_logger().info("At seam END: waiting (cool-down)")
        _sleep_with_spin(self, 1.0)

        # Retract
        retract = _pose_xyz_tool_down(seam_points[0][0], seam_points[0][1], 0.32, yaw_rad=0.0)
        if not self.send_lin_with_retries(retract):
            return 1

        self.get_logger().info("=== WELDING DEMO: done ===")
        return 0


def main(argv: Optional[List[str]] = None) -> None:
    rclpy.init(args=argv)
    node = WeldingDemo()
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
