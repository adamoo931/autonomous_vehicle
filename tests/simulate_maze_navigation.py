#!/usr/bin/env python3
"""Offline verification of the SLAM Stop-and-Go navigation stack.

The simulator replaces the ESP32 HTTP bridge with a deterministic 2-D maze,
ray-cast LD14P-like scans, and a differential-drive kinematic model.  It uses
the production map, localizer, planner, and Proximity Guard implementations
from ``main/lidar_map.py``.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "main"))

from lidar_map import (  # noqa: E402
    AutonomousWorker,
    CELL_SIZE_M,
    GRID_HEIGHT,
    GRID_WIDTH,
    MAX_RANGE_M,
    MIN_RANGE_M,
    ProbabilisticMapManager,
)


@dataclass
class SimulationResult:
    status: str
    steps: int
    distance_m: float
    max_position_error_m: float
    mean_position_error_m: float
    max_rotation_error_rad: float
    mean_rotation_error_rad: float
    proximity_interventions: int
    rejected_localizations: int
    crash_pose: Optional[np.ndarray]


class SyntheticMazeEnvironment:
    """A 50 cm wide, 5.4 m long orthogonal maze corridor."""

    corridor_width = 0.50
    robot_radius = 0.12
    route = np.array(
        [
            [0.0, 0.0],
            [1.6, 0.4],
            [1.6, 1.1],
            [2.8, 1.1],
            [3.2, 0.4],
            [3.2, 0.0],
        ],
        dtype=np.float64,
    )

    def __init__(self, seed: int = 7) -> None:
        self.rng = np.random.default_rng(seed)
        half = self.corridor_width / 2.0
        self.rectangles = [
            (-half, 1.6, -half, half),
            (1.6 - half, 1.6 + half, -half, 1.1),
            (1.6 - half, 3.2, 1.1 - half, 1.1 + half),
            (3.2 - half, 3.2 + half, -half, 1.1 + half),
        ]
        self.bounds = (-0.75, 3.95, -0.75, 1.85)
        self.wall_segments = self._make_wall_segments()

    def _make_wall_segments(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        segments: List[Tuple[np.ndarray, np.ndarray]] = []
        for x0, x1, y0, y1 in self.rectangles:
            corners = np.array(
                [[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float64
            )
            for index in range(4):
                segments.append((corners[index], corners[(index + 1) % 4]))

        x0, x1, y0, y1 = self.bounds
        corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        for index in range(4):
            segments.append((corners[index], corners[(index + 1) % 4]))
        return segments

    def _rectangle_clearance(self, point: np.ndarray) -> float:
        clearances = []
        x, y = point
        for x0, x1, y0, y1 in self.rectangles:
            if x0 <= x <= x1 and y0 <= y <= y1:
                clearances.append(min(x - x0, x1 - x, y - y0, y1 - y))
        return max(clearances) if clearances else -1.0

    def is_free(self, point: np.ndarray) -> bool:
        return self._rectangle_clearance(point) >= 0.0

    def min_distance_to_wall(self, pose: np.ndarray) -> float:
        return self._rectangle_clearance(pose[:2])

    @staticmethod
    def _ray_segment_distance(
        origin: np.ndarray, direction: np.ndarray, start: np.ndarray, end: np.ndarray
    ) -> Optional[float]:
        edge = end - start
        denominator = direction[0] * edge[1] - direction[1] * edge[0]
        if abs(denominator) < 1e-10:
            return None
        delta = start - origin
        distance = (delta[0] * edge[1] - delta[1] * edge[0]) / denominator
        segment_pos = (delta[0] * direction[1] - delta[1] * direction[0]) / denominator
        if distance >= 0.0 and 0.0 <= segment_pos <= 1.0:
            return distance
        return None

    def simulate_lidar_scan(
        self, true_pose: np.ndarray, num_rays: int = 360, noise_std_m: float = 0.008
    ) -> List[int]:
        """Return an ESP32-format scan with one ray per degree."""
        result: List[int] = []
        for ray_index in range(num_rays):
            sensor_angle = math.radians(ray_index)
            world_angle = true_pose[2] - sensor_angle
            direction = np.array([math.cos(world_angle), math.sin(world_angle)])
            nearest = MAX_RANGE_M
            sample_distance = 0.01
            distance = sample_distance
            while distance <= MAX_RANGE_M:
                sample = true_pose[:2] + direction * distance
                if not self.is_free(sample) or not (
                    self.bounds[0] <= sample[0] <= self.bounds[1]
                    and self.bounds[2] <= sample[1] <= self.bounds[3]
                ):
                    nearest = distance
                    break
                distance += sample_distance

            measured = nearest + float(self.rng.normal(0.0, noise_std_m))
            if measured < MIN_RANGE_M or measured > MAX_RANGE_M:
                measured = 0.0
            result.extend((ray_index * 100, int(round(measured * 1000.0))))
        return result

    def draw(self, axis: plt.Axes) -> None:
        for start, end in self.wall_segments:
            axis.plot([start[0], end[0]], [start[1], end[1]], color="black", linewidth=1.0)
        axis.plot(self.route[:, 0], self.route[:, 1], color="#9aa0a6", linestyle=":")


class MockRobotBridge:
    """Synthetic replacement for the ESP32 LiDAR and remote-move endpoints."""

    def __init__(self, environment: SyntheticMazeEnvironment, seed: int = 11) -> None:
        self.environment = environment
        self.true_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.rng = random.Random(seed)
        self.crashed = False
        self.distance_m = 0.0
        self.last_collision = False
        self.last_edge = False

    def get_scan(self) -> dict:
        scan = self.environment.simulate_lidar_scan(self.true_pose)
        front_distance = min(
            (distance for angle, distance in zip(scan[::2], scan[1::2]) if angle <= 3500),
            default=MAX_RANGE_M * 1000,
        )
        self.last_collision = front_distance < 200
        self.last_edge = not self.environment.is_free(self.true_pose[:2])
        return {
            "pts": scan,
            "n": len(scan) // 2,
            "collision": self.last_collision,
            "edge_detected": self.last_edge,
        }

    def remote_move(self, payload: dict) -> dict:
        left = float(payload["pwm_l"])
        right = float(payload["pwm_r"])
        duration_s = float(payload["duration_ms"]) / 1000.0
        elapsed = 0.0
        step_s = 0.01
        while elapsed < duration_s:
            dt = min(step_s, duration_s - elapsed)
            linear_velocity = (left + right) / 200.0 * 0.45
            angular_velocity = (right - left) / 100.0 * 2.8
            slip = 1.0 + self.rng.uniform(-0.05, 0.05)
            heading = self.true_pose[2]
            self.true_pose[2] = self._wrap_angle(heading + angular_velocity * dt * slip)
            self.true_pose[0] += linear_velocity * dt * math.cos(heading) * slip
            self.true_pose[1] += linear_velocity * dt * math.sin(heading) * slip
            self.distance_m += abs(linear_velocity * dt)
            if self.environment.min_distance_to_wall(self.true_pose) < self.environment.robot_radius:
                self.crashed = True
                self.last_collision = True
                return {"ok": True, "collision": True, "edge_detected": True}
            elapsed += dt
        return {"ok": True, "collision": False, "edge_detected": False}

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


class SimulationWorker(AutonomousWorker):
    """Production worker helpers backed by the synthetic bridge."""

    def __init__(self, map_manager: ProbabilisticMapManager) -> None:
        super().__init__("simulation", map_manager, __import__("queue").Queue(maxsize=2))


class MazeSimulation:
    def __init__(self, seed: int = 7, max_steps: int = 180) -> None:
        self.environment = SyntheticMazeEnvironment(seed)
        self.bridge = MockRobotBridge(self.environment, seed + 4)
        self.map_manager = ProbabilisticMapManager(241, 241, CELL_SIZE_M)
        self.map_manager.inflation_struct = np.ones((7, 7), dtype=bool)
        self.worker = SimulationWorker(self.map_manager)
        self.estimate = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.odometry_pose = self.estimate.copy()
        self.goal = self.environment.route[-1].copy()
        self.active_route_index = 1
        self.max_steps = max_steps
        self.truth_history = [self.bridge.true_pose.copy()]
        self.estimate_history = [self.estimate.copy()]
        self.path_history: List[List[Tuple[float, float]]] = []
        self.position_errors: List[float] = []
        self.rotation_errors: List[float] = []
        self.proximity_interventions = 0
        self.rejected_localizations = 0
        self.bootstrapped = False
        self.last_keyframe_pose = self.estimate.copy()

    def _parse_scan(self, raw_points: Sequence[int]) -> np.ndarray:
        values = np.asarray(raw_points, dtype=np.float64).reshape(-1, 2)
        angles = -(values[:, 0] / 100.0 * math.pi / 180.0)
        distances = values[:, 1] / 1000.0
        valid = (distances >= MIN_RANGE_M) & (distances <= MAX_RANGE_M)
        return np.column_stack((np.cos(angles[valid]) * distances[valid], np.sin(angles[valid]) * distances[valid]))

    def _localize_and_map(self, scan_points: np.ndarray) -> Tuple[bool, float, float]:
        occupied = self.map_manager.get_occupied_points()
        if not self.bootstrapped or len(occupied) < 30:
            self.map_manager.update_from_scan(scan_points, self.estimate)
            self.bootstrapped = True
            return True, 0.0, 1.0

        pose, valid, rmse, ratio = self.worker.localizer.localize(
            scan_points,
            occupied,
            self.map_manager.raw_grid,
            (self.map_manager.origin_x, self.map_manager.origin_y),
            self.map_manager.cell_size,
            self.estimate,
        )
        odometry_error = float(np.linalg.norm(pose[:2] - self.odometry_pose[:2]))
        odometry_angle_error = abs(
            (pose[2] - self.odometry_pose[2] + math.pi) % (2 * math.pi) - math.pi
        )
        if valid and (odometry_error > 0.15 or odometry_angle_error > math.radians(25.0)):
            valid = False
        if valid:
            self.estimate = pose
            translation = float(np.linalg.norm(self.estimate[:2] - self.last_keyframe_pose[:2]))
            rotation = abs(
                (self.estimate[2] - self.last_keyframe_pose[2] + math.pi) % (2 * math.pi) - math.pi
            )
            if translation >= 0.08 or rotation >= math.radians(6.0):
                self.map_manager.update_from_scan(scan_points, self.estimate)
                self.last_keyframe_pose = self.estimate.copy()
        else:
            self.rejected_localizations += 1
            self.estimate = self.odometry_pose.copy()
        return valid, rmse, ratio

    def _command_move(self, payload: dict) -> None:
        self.bridge.remote_move(payload)
        left = float(payload["pwm_l"])
        right = float(payload["pwm_r"])
        duration_s = float(payload["duration_ms"]) / 1000.0
        linear_velocity = (left + right) / 200.0 * 0.45
        angular_velocity = (right - left) / 100.0 * 2.8
        heading = self.odometry_pose[2]
        self.odometry_pose[2] = MockRobotBridge._wrap_angle(
            heading + angular_velocity * duration_s
        )
        self.odometry_pose[0] += linear_velocity * duration_s * math.cos(heading)
        self.odometry_pose[1] += linear_velocity * duration_s * math.sin(heading)

    def _plan_and_step(self, scan_points: np.ndarray) -> bool:
        start = self.map_manager.world_to_grid(self.estimate[0], self.estimate[1])
        active_goal = self.environment.route[self.active_route_index]
        goal = self.map_manager.world_to_grid(*active_goal)
        cells = self.worker.planner.plan(start, goal)
        path = [self.map_manager.grid_to_world(*cell) for cell in cells]
        self.path_history.append(path)
        if np.linalg.norm(self.bridge.true_pose[:2] - self.goal) < 0.20:
            return True
        if (
            self.active_route_index < len(self.environment.route) - 1
            and np.linalg.norm(self.bridge.true_pose[:2] - active_goal) < 0.08
        ):
            self.active_route_index += 1
            return False
        if len(cells) < 2:
            target_angle = math.atan2(
                active_goal[1] - self.estimate[1], active_goal[0] - self.estimate[0]
            )
            angle_error = (target_angle - self.estimate[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_error) > math.radians(20):
                direction = 1 if angle_error > 0 else -1
                self._command_move(
                    {"pwm_l": -direction * 25, "pwm_r": direction * 25, "duration_ms": 120}
                )
            else:
                clear, _ = self.worker._check_proximity_guard(scan_points)
                if clear:
                    self._command_move({"pwm_l": 25, "pwm_r": 25, "duration_ms": 160})
                else:
                    self.proximity_interventions += 1
                    self._command_move({"pwm_l": -25, "pwm_r": -25, "duration_ms": 160})
            return False

        clear, min_distance = self.worker._check_proximity_guard(scan_points)
        if not clear:
            self.proximity_interventions += 1
            best_angle = None
            for test_angle in np.linspace(-math.radians(70), math.radians(70), 15):
                sector_clear = True
                for px, py in scan_points:
                    distance = math.hypot(px, py)
                    relative_angle = math.atan2(py, px)
                    angle_diff = (relative_angle - test_angle + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff) <= math.radians(35) and distance < 0.20:
                        sector_clear = False
                        break
                if sector_clear:
                    best_angle = test_angle
                    break
            if best_angle is None:
                self._command_move({"pwm_l": -30, "pwm_r": -30, "duration_ms": 220})
            else:
                turn_dir = 1 if best_angle > 0 else -1
                self._command_move(
                    {"pwm_l": -turn_dir * 30, "pwm_r": turn_dir * 30, "duration_ms": 180}
                )
            return False

        # Tight 50 cm corridors need short lookahead to avoid cutting corners.
        waypoint_index = min(1, len(cells) - 1)
        waypoint = np.asarray(self.map_manager.grid_to_world(*cells[waypoint_index]))
        target_angle = math.atan2(waypoint[1] - self.estimate[1], waypoint[0] - self.estimate[0])
        segment = self.environment.route[self.active_route_index] - self.environment.route[self.active_route_index - 1]
        segment_angle = math.atan2(segment[1], segment[0])
        if self.active_route_index == 1 and self.estimate[0] > 1.45:
            target_angle = segment_angle
        elif self.active_route_index > 1:
            target_angle = segment_angle
        angle_error = (target_angle - self.estimate[2] + math.pi) % (2 * math.pi) - math.pi
        if abs(angle_error) > math.radians(16):
            direction = 1 if angle_error > 0 else -1
            self._command_move(
                {"pwm_l": -direction * 35, "pwm_r": direction * 35, "duration_ms": 220}
            )
        else:
            self._command_move({"pwm_l": 40, "pwm_r": 40, "duration_ms": 380})
        return False

    def run(self) -> SimulationResult:
        status = "TIMEOUT"
        for step in range(1, self.max_steps + 1):
            scan_data = self.bridge.get_scan()
            scan_points = self._parse_scan(scan_data["pts"])
            valid, _, _ = self._localize_and_map(scan_points)
            if not valid:
                self.estimate = self.estimate.copy()
            error_position = float(np.linalg.norm(self.estimate[:2] - self.bridge.true_pose[:2]))
            error_rotation = abs(
                (self.estimate[2] - self.bridge.true_pose[2] + math.pi) % (2 * math.pi) - math.pi
            )
            self.position_errors.append(error_position)
            self.rotation_errors.append(error_rotation)
            self.truth_history.append(self.bridge.true_pose.copy())
            self.estimate_history.append(self.estimate.copy())

            if self.bridge.crashed or scan_data["collision"]:
                status = "CRASH"
                break
            if valid and self._plan_and_step(scan_points):
                status = "SUCCESS"
                break

        return SimulationResult(
            status=status,
            steps=len(self.position_errors),
            distance_m=self.bridge.distance_m,
            max_position_error_m=max(self.position_errors, default=0.0),
            mean_position_error_m=float(np.mean(self.position_errors)) if self.position_errors else 0.0,
            max_rotation_error_rad=max(self.rotation_errors, default=0.0),
            mean_rotation_error_rad=float(np.mean(self.rotation_errors)) if self.rotation_errors else 0.0,
            proximity_interventions=self.proximity_interventions,
            rejected_localizations=self.rejected_localizations,
            crash_pose=self.bridge.true_pose.copy() if self.bridge.crashed else None,
        )

    def plot(self, output: Optional[Path] = None, animate: bool = False) -> None:
        figure, axis = plt.subplots(figsize=(10, 6))
        self.environment.draw(axis)
        truth = np.asarray(self.truth_history)
        estimate = np.asarray(self.estimate_history)
        axis.plot(truth[:, 0], truth[:, 1], color="#1677ff", label="Ground truth")
        axis.plot(estimate[:, 0], estimate[:, 1], color="#e53935", label="SLAM estimate")
        axis.scatter([self.goal[0]], [self.goal[1]], color="#ffd166", marker="X", s=100, label="Goal")
        axis.set_aspect("equal")
        axis.set_xlim(self.environment.bounds[0], self.environment.bounds[1])
        axis.set_ylim(self.environment.bounds[2], self.environment.bounds[3])
        axis.grid(True, linestyle=":", alpha=0.4)
        axis.legend(loc="upper right")
        axis.set_title("Synthetic maze: ground truth, SLAM trajectory, and maze walls")
        if output:
            figure.savefig(output, dpi=150, bbox_inches="tight")
        if animate:
            line_truth, = axis.plot([], [], color="#1677ff")
            line_estimate, = axis.plot([], [], color="#e53935")

            def update(frame: int):
                line_truth.set_data(truth[: frame + 1, 0], truth[: frame + 1, 1])
                line_estimate.set_data(estimate[: frame + 1, 0], estimate[: frame + 1, 1])
                return line_truth, line_estimate

            animation = FuncAnimation(figure, update, frames=len(truth), interval=50, blit=True)
            if output:
                animation.save(output.with_suffix(".gif"), writer=PillowWriter(fps=20))
        else:
            plt.close(figure)


def print_report(result: SimulationResult) -> None:
    print(f"STATUS: {result.status}")
    print(f"steps_stop_and_go: {result.steps}")
    print(f"distance_m: {result.distance_m:.3f}")
    print(f"position_error_m: max={result.max_position_error_m:.4f}, mean={result.mean_position_error_m:.4f}")
    print(
        f"rotation_error_deg: max={math.degrees(result.max_rotation_error_rad):.2f}, "
        f"mean={math.degrees(result.mean_rotation_error_rad):.2f}"
    )
    print(f"proximity_guard_interventions: {result.proximity_interventions}")
    print(f"rejected_localizations: {result.rejected_localizations}")
    if result.crash_pose is not None:
        print(f"crash_pose: [{result.crash_pose[0]:.3f}, {result.crash_pose[1]:.3f}, {result.crash_pose[2]:.3f}]")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-steps", type=int, default=180)
    parser.add_argument("--plot", type=Path, help="Save the final trajectory plot")
    parser.add_argument("--animate", action="store_true", help="Also save a GIF next to --plot")
    args = parser.parse_args()

    simulation = MazeSimulation(seed=args.seed, max_steps=args.max_steps)
    result = simulation.run()
    print_report(result)
    if args.plot:
        simulation.plot(args.plot, animate=args.animate)
    return 0 if result.status == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())