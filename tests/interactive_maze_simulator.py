#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Real-Time SLAM & Stop-and-Go Navigation Simulator (6x6m Canvas).
Samodzielny plik testowy z bezpośrednią integracją algorytmów CSM + SVD-ICP + A*.
"""

from __future__ import annotations

import math
import os
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

os.environ["MATPLOTLIBRC"] = ""
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.style as mplstyle
mplstyle.use("fast")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from scipy.linalg import svd
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree

# Konfiguracja Środowiska i Siatki
SIM_CANVAS_M = 6.0              # 6x6 m
CELL_SIZE_M = 0.05               # 5 cm na komórkę SLAM
GRID_WIDTH = 401
GRID_HEIGHT = 401
MIN_RANGE_M = 0.08
MAX_RANGE_M = 5.5
PROXIMITY_GUARD_M = 0.28         # Bufor 28 cm do przeszkód
PROXIMITY_GUARD_ANGLE = math.radians(45)

SIM_GRID_RES_M = 0.03            # Rozdzielczość fizycznego świata
SIM_GRID_DIM = int(SIM_CANVAS_M / SIM_GRID_RES_M) + 1
ROBOT_COLLISION_RADIUS_M = 0.12  # Promień obrysu podwozia [m]


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    points: List[Tuple[int, int]] = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points


class ProbabilisticMapManager:
    def __init__(self, width: int, height: int, cell_size: float) -> None:
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.raw_grid = np.full((height, width), -1, dtype=np.int8)
        self.origin_x = width // 2
        self.origin_y = height // 2
        # Bufor bezpieczeństwa A*: 4 komórki = 20 cm
        self.inflation_struct = np.ones((9, 9), dtype=bool)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ix = int(math.floor(x / self.cell_size)) + self.origin_x
        iy = int(math.floor(y / self.cell_size)) + self.origin_y
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        x = (ix - self.origin_x + 0.5) * self.cell_size
        y = (iy - self.origin_y + 0.5) * self.cell_size
        return x, y

    def is_in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def update_from_scan(self, scan_points: np.ndarray, pose: np.ndarray) -> None:
        rx, ry = self.world_to_grid(pose[0], pose[1])
        c_th, s_th = math.cos(pose[2]), math.sin(pose[2])

        for px, py in scan_points:
            dist = math.hypot(px, py)
            if dist < MIN_RANGE_M or dist > MAX_RANGE_M:
                continue
            wx = pose[0] + c_th * px - s_th * py
            wy = pose[1] + s_th * px + c_th * py
            gx, gy = self.world_to_grid(wx, wy)

            if self.is_in_bounds(gx, gy):
                line = bresenham_line(rx, ry, gx, gy)
                for lx, ly in line[:-1]:
                    if self.is_in_bounds(lx, ly) and self.raw_grid[ly, lx] != 100:
                        self.raw_grid[ly, lx] = 0
                self.raw_grid[gy, gx] = 100

    def get_occupied_points(self, max_points: int = 3000) -> np.ndarray:
        occupied = np.argwhere(self.raw_grid == 100)
        if len(occupied) == 0:
            return np.empty((0, 2), dtype=np.float64)
        if len(occupied) > max_points:
            indices = np.random.choice(len(occupied), max_points, replace=False)
            occupied = occupied[indices]
        pts = np.empty((len(occupied), 2), dtype=np.float64)
        pts[:, 0] = (occupied[:, 1] - self.origin_x + 0.5) * self.cell_size
        pts[:, 1] = (occupied[:, 0] - self.origin_y + 0.5) * self.cell_size
        return pts

    def get_inflated_costmap(self) -> np.ndarray:
        occupied = (self.raw_grid == 100)
        return binary_dilation(occupied, structure=self.inflation_struct, border_value=0)

    def to_image(self) -> np.ndarray:
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[self.raw_grid == -1] = (16, 20, 26)
        img[self.raw_grid == 0] = (220, 226, 220)
        img[self.raw_grid == 100] = (15, 15, 15)
        return img


class RobustLocalizer:
    def __init__(self, max_iter: int = 10, tol: float = 1e-4) -> None:
        self.max_iter = max_iter
        self.tol = tol

    @staticmethod
    def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        return (points @ rot.T) + pose[:2]

    def coarse_angle_search(self, scan_pts: np.ndarray, raw_grid: np.ndarray,
                            origin_xy: Tuple[int, int], cell_size: float,
                            base_pose: np.ndarray) -> float:
        center_deg = math.degrees(base_pose[2])
        angles = np.radians(np.arange(center_deg - 30, center_deg + 31, 2.0))
        best_angle = base_pose[2]
        best_score = -1
        ox, oy = origin_xy
        h, w = raw_grid.shape
        sample = scan_pts[::2]

        for th in angles:
            c, s = math.cos(th), math.sin(th)
            wx = base_pose[0] + c * sample[:, 0] - s * sample[:, 1]
            wy = base_pose[1] + s * sample[:, 0] + c * sample[:, 1]
            ixs = np.floor(wx / cell_size).astype(np.int32) + ox
            iys = np.floor(wy / cell_size).astype(np.int32) + oy
            valid = (ixs >= 0) & (ixs < w) & (iys >= 0) & (iys < h)
            if not np.any(valid):
                continue
            score = int(np.sum(raw_grid[iys[valid], ixs[valid]] == 100))
            if score > best_score:
                best_score = score
                best_angle = th
        return best_angle

    def localize(self, scan_pts: np.ndarray, map_pts: np.ndarray,
                 raw_grid: np.ndarray, origin_xy: Tuple[int, int],
                 cell_size: float, current_pose: np.ndarray) -> Tuple[np.ndarray, bool, float, float]:
        if len(scan_pts) < 15 or len(map_pts) < 15:
            return current_pose.copy(), False, 999.0, 0.0

        best_th = self.coarse_angle_search(scan_pts, raw_grid, origin_xy, cell_size, current_pose)
        pose = np.array([current_pose[0], current_pose[1], best_th], dtype=np.float64)

        tree = cKDTree(map_pts)
        final_rmse = 999.0
        inliers_count = 0

        for _ in range(self.max_iter):
            transformed = self.transform_points(scan_pts, pose)
            dists, idxs = tree.query(transformed, k=1, distance_upper_bound=0.22)
            valid = dists < 0.22
            inliers_count = int(np.count_nonzero(valid))
            if inliers_count < 20:
                break

            src = transformed[valid]
            dst = map_pts[idxs[valid]]
            mean_s = np.mean(src, axis=0)
            mean_d = np.mean(dst, axis=0)

            H = (src - mean_s).T @ (dst - mean_d)
            U, _, Vt = svd(H)
            R_delta = Vt.T @ U.T
            if np.linalg.det(R_delta) < 0:
                Vt[1, :] *= -1
                R_delta = Vt.T @ U.T

            t_delta = mean_d - R_delta @ mean_s
            pose[:2] = R_delta @ pose[:2] + t_delta
            d_theta = math.atan2(R_delta[1, 0], R_delta[0, 0])
            pose[2] = (pose[2] + d_theta + math.pi) % (2 * math.pi) - math.pi
            final_rmse = float(np.sqrt(np.mean(dists[valid] ** 2)))

            if np.linalg.norm(t_delta) < self.tol and abs(d_theta) < self.tol:
                break

        inlier_ratio = float(inliers_count / len(scan_pts)) if len(scan_pts) > 0 else 0.0
        is_valid = (final_rmse <= 0.085 and inlier_ratio >= 0.35 and inliers_count >= 20)
        return (pose if is_valid else current_pose.copy()), is_valid, final_rmse, inlier_ratio


class OptimisticPathPlanner:
    def __init__(self, map_manager: ProbabilisticMapManager) -> None:
        self.map_mgr = map_manager

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        if not self.map_mgr.is_in_bounds(start[0], start[1]) or not self.map_mgr.is_in_bounds(goal[0], goal[1]):
            return []

        costmap_inflated = self.map_mgr.get_inflated_costmap()
        if costmap_inflated[goal[1], goal[0]]:
            found = False
            for r in range(1, 10):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        gx, gy = goal[0] + dx, goal[1] + dy
                        if self.map_mgr.is_in_bounds(gx, gy) and not costmap_inflated[gy, gx]:
                            goal = (gx, gy)
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                return []

        open_set = [(0.0, start)]
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        g_score = {start: 0.0}

        while open_set:
            _, curr = min(open_set, key=lambda it: it[0])
            open_set = [it for it in open_set if it[1] != curr]

            if curr == goal:
                path = []
                while curr:
                    path.append(curr)
                    curr = came_from[curr]
                return list(reversed(path))

            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = curr[0] + dx, curr[1] + dy
                if not self.map_mgr.is_in_bounds(nx, ny) or costmap_inflated[ny, nx]:
                    continue

                step_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                unknown_penalty = 1.4 if self.map_mgr.raw_grid[ny, nx] == -1 else 1.0
                tentative = g_score[curr] + step_cost * unknown_penalty

                if tentative < g_score.get((nx, ny), float("inf")):
                    came_from[(nx, ny)] = curr
                    g_score[(nx, ny)] = tentative
                    f_score = tentative + math.hypot(nx - goal[0], ny - goal[1])
                    open_set.append((f_score, (nx, ny)))
        return []


class VirtualEnvironment:
    def __init__(self, dim: int, res: float) -> None:
        self.dim = dim
        self.res = res
        self.origin_idx = dim // 2
        self.grid = np.zeros((dim, dim), dtype=bool)
        self.grid[0:2, :] = True
        self.grid[-2:, :] = True
        self.grid[:, 0:2] = True
        self.grid[:, -2:] = True

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return int(round(x / self.res)) + self.origin_idx, int(round(y / self.res)) + self.origin_idx

    def add_brush_stroke(self, wx: float, wy: float, radius_m: float = 0.10) -> None:
        cx, cy = self.world_to_grid(wx, wy)
        r_cells = int(math.ceil(radius_m / self.res))
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy <= r_cells * r_cells:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.dim and 0 <= ny < self.dim:
                        self.grid[ny, nx] = True

    def clear(self) -> None:
        self.grid.fill(False)
        self.grid[0:2, :] = True
        self.grid[-2:, :] = True
        self.grid[:, 0:2] = True
        self.grid[:, -2:] = True

    def ray_cast_scan(self, true_pose: np.ndarray) -> np.ndarray:
        angles = -(np.arange(360) * math.pi / 180.0)
        rx, ry, r_theta = true_pose[0], true_pose[1], true_pose[2]
        dists = np.full(360, MAX_RANGE_M, dtype=np.float64)
        ds = 0.03
        max_steps = int(MAX_RANGE_M / ds)

        for deg in range(360):
            ray_dir = r_theta - math.radians(deg)
            c, s = math.cos(ray_dir), math.sin(ray_dir)
            for step in range(1, max_steps):
                dist = step * ds
                ix, iy = self.world_to_grid(rx + c * dist, ry + s * dist)
                if not (0 <= ix < self.dim and 0 <= iy < self.dim) or self.grid[iy, ix]:
                    dists[deg] = dist + np.random.normal(0.0, 0.005)
                    break

        valid = (dists >= MIN_RANGE_M) & (dists <= MAX_RANGE_M)
        return np.column_stack((np.cos(angles[valid]) * dists[valid], np.sin(angles[valid]) * dists[valid]))

    def check_collision(self, wx: float, wy: float) -> bool:
        ix, iy = self.world_to_grid(wx, wy)
        r_cells = int(math.ceil(ROBOT_COLLISION_RADIUS_M / self.res))
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy <= r_cells * r_cells:
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < self.dim and 0 <= ny < self.dim and self.grid[ny, nx]:
                        return True
        return False


class SimulationEngine(threading.Thread):
    def __init__(self, env: VirtualEnvironment, out_queue: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.env = env
        self.out_queue = out_queue
        self.map_mgr = ProbabilisticMapManager(GRID_WIDTH, GRID_HEIGHT, CELL_SIZE_M)
        self.localizer = RobustLocalizer()
        self.planner = OptimisticPathPlanner(self.map_mgr)

        self.true_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.goal_world: Optional[np.ndarray] = None
        self.nav_enabled = False
        self.running = True
        self.crashed = False

    def reset_pose(self, pos: np.ndarray) -> None:
        self.true_pose = pos.copy()
        self.pose = pos.copy()
        self.crashed = False
        self.map_mgr = ProbabilisticMapManager(GRID_WIDTH, GRID_HEIGHT, CELL_SIZE_M)
        self.planner.map_mgr = self.map_mgr

    def run(self) -> None:
        bootstrapped = False
        last_kf_pose = self.pose.copy()

        while self.running:
            scan_pts = self.env.ray_cast_scan(self.true_pose)
            map_pts = self.map_mgr.get_occupied_points()

            if not bootstrapped or len(map_pts) < 30:
                self.map_mgr.update_from_scan(scan_pts, self.pose)
                last_kf_pose = self.pose.copy()
                bootstrapped = True
                is_valid, rmse, ratio = True, 0.0, 1.0
                state_msg = "BOOTSTRAP OK"
            else:
                origin = (self.map_mgr.origin_x, self.map_mgr.origin_y)
                new_pose, is_valid, rmse, ratio = self.localizer.localize(
                    scan_pts, map_pts, self.map_mgr.raw_grid, origin,
                    self.map_mgr.cell_size, self.pose
                )
                if is_valid:
                    self.pose = new_pose
                    d_t = np.linalg.norm(self.pose[:2] - last_kf_pose[:2])
                    d_r = abs((self.pose[2] - last_kf_pose[2] + math.pi) % (2 * math.pi) - math.pi)
                    if d_t >= 0.08 or d_r >= math.radians(6.0):
                        self.map_mgr.update_from_scan(scan_pts, self.pose)
                        last_kf_pose = self.pose.copy()
                    state_msg = "SLAM OK"
                else:
                    state_msg = f"ICP REJECTED (RMSE={rmse:.3f})"

            path_cells = []
            if self.goal_world is not None and is_valid:
                start_c = self.map_mgr.world_to_grid(self.pose[0], self.pose[1])
                goal_c = self.map_mgr.world_to_grid(self.goal_world[0], self.goal_world[1])
                path_cells = self.planner.plan(start_c, goal_c)

                if self.nav_enabled and len(path_cells) >= 2:
                    dist_to_goal = float(np.linalg.norm(self.pose[:2] - self.goal_world))
                    if dist_to_goal < 0.20:
                        state_msg = "GOAL REACHED!"
                        self.nav_enabled = False
                    else:
                        # Proximity Guard na lokalnej chmurze
                        is_clear = True
                        for px, py in scan_pts:
                            d = math.hypot(px, py)
                            ang = math.atan2(py, px)
                            if abs(ang) <= PROXIMITY_GUARD_ANGLE and d < PROXIMITY_GUARD_M:
                                is_clear = False
                                break

                        if not is_clear:
                            state_msg = "PROXIMITY GUARD: Korygowanie..."
                            # Wykonaj manewr wycofania/obrotu w symulacji fizycznej
                            self.true_pose[2] += math.radians(20.0)
                            self.true_pose[0] -= 0.04 * math.cos(self.true_pose[2])
                            self.true_pose[1] -= 0.04 * math.sin(self.true_pose[2])
                        else:
                            # Nawigacja wzdłuż A* z lookaheadem 4 komórek (20 cm)
                            t_idx = min(4, len(path_cells) - 1)
                            target_wx, target_wy = self.map_mgr.grid_to_world(*path_cells[t_idx])
                            d_x, d_y = target_wx - self.pose[0], target_wy - self.pose[1]
                            target_ang = math.atan2(d_y, d_x)
                            ang_err = (target_ang - self.pose[2] + math.pi) % (2 * math.pi) - math.pi

                            if abs(ang_err) > math.radians(16.0):
                                rot_step = math.radians(18.0) if ang_err > 0 else -math.radians(18.0)
                                self.true_pose[2] += rot_step * (1.0 + np.random.uniform(-0.02, 0.02))
                                state_msg = f"OBROT: {math.degrees(ang_err):.0f}°"
                            else:
                                step_dist = 0.08 * (1.0 + np.random.uniform(-0.02, 0.02))
                                nx = self.true_pose[0] + step_dist * math.cos(self.true_pose[2])
                                ny = self.true_pose[1] + step_dist * math.sin(self.true_pose[2])
                                if self.env.check_collision(nx, ny):
                                    self.crashed = True
                                else:
                                    self.crashed = False
                                    self.true_pose[0] = nx
                                    self.true_pose[1] = ny
                                state_msg = "JAZDA WPROST"

            scan_world = self.localizer.transform_points(scan_pts, self.pose)
            path_pts = [self.map_mgr.grid_to_world(*c) for c in path_cells]

            snapshot = {
                "pose": self.pose.copy(),
                "true_pose": self.true_pose.copy(),
                "map_image": self.map_mgr.to_image(),
                "scan_world": scan_world,
                "path_pts": path_pts,
                "state_msg": state_msg,
                "rmse": rmse,
                "crashed": self.crashed
            }

            if self.out_queue.full():
                try:
                    self.out_queue.get_nowait()
                except queue.Empty:
                    pass
            self.out_queue.put_nowait(snapshot)
            time.sleep(0.04)


class InteractiveSimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Interactive Real-Time SLAM Navigation Simulator (6x6m)")
        self.root.geometry("1400x900")

        self.env = VirtualEnvironment(SIM_GRID_DIM, SIM_GRID_RES_M)
        self.data_queue: queue.Queue = queue.Queue(maxsize=2)
        self.engine = SimulationEngine(self.env, self.data_queue)

        self.mode = tk.StringVar(value="WALL")
        self.gt_history: List[Tuple[float, float]] = []

        self._build_ui()
        self.engine.start()
        self.root.after(40, self._poll_simulation)

    def _build_ui(self) -> None:
        ctrl = ttk.Frame(self.root, padding=8)
        ctrl.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(ctrl, text="Tryb myszy:").pack(side=tk.LEFT, padx=4)
        for label, val in [("✏️ Rysuj Ściany", "WALL"), ("🤖 Ustaw Robota", "SET_ROBOT"), ("🎯 Ustaw Cel", "SET_GOAL")]:
            ttk.Radiobutton(ctrl, text=label, variable=self.mode, value=val).pack(side=tk.LEFT, padx=4)

        ttk.Separator(ctrl, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        self.btn_nav = ttk.Button(ctrl, text="🚀 JEDŹ DO CELU", command=self.toggle_navigation)
        self.btn_nav.pack(side=tk.LEFT, padx=6)
        ttk.Button(ctrl, text="🗑️ Wyczyść Ściany", command=self.clear_walls).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="🔄 Reset Mapy", command=self.reset_map).pack(side=tk.LEFT, padx=4)

        self.lbl_status = ttk.Label(ctrl, text="Status: Gotowy", font=("Consolas", 10))
        self.lbl_status.pack(side=tk.LEFT, padx=12)

        self.fig = Figure(figsize=(13.5, 7.5), dpi=100)
        self.ax_sim = self.fig.add_subplot(121)
        self.ax_slam = self.fig.add_subplot(122)

        for ax, title in [(self.ax_sim, "Świat Fizyczny (Rysuj myszką)"), (self.ax_slam, "Mapa SLAM (Occupancy Grid + A*)")]:
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlim(-3.0, 3.0)
            ax.set_ylim(-3.0, 3.0)
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.set_title(title)
            ax.grid(True, linestyle=":", alpha=0.4)

        self.img_env = self.ax_sim.imshow(self.env.grid, cmap="gray_r", extent=[-3.0, 3.0, -3.0, 3.0], origin="lower")
        self.line_gt, = self.ax_sim.plot([], [], color="#1677ff", linewidth=1.5, label="Ground Truth")
        self.marker_gt, = self.ax_sim.plot([0], [0], "o", color="#1677ff", markersize=10)
        self.marker_goal_sim, = self.ax_sim.plot([], [], "X", color="#ffd166", markersize=12)
        self.ax_sim.legend(loc="upper right", fontsize=8)

        self.img_slam = self.ax_slam.imshow(
            self.engine.map_mgr.to_image(),
            extent=[-GRID_WIDTH // 2 * CELL_SIZE_M, GRID_WIDTH // 2 * CELL_SIZE_M,
                    -GRID_HEIGHT // 2 * CELL_SIZE_M, GRID_HEIGHT // 2 * CELL_SIZE_M],
            origin="lower"
        )
        self.scatter_scan, = self.ax_slam.plot([], [], "o", color="#06d6a0", markersize=2, alpha=0.5)
        self.line_path, = self.ax_slam.plot([], [], color="#ffd166", linewidth=2.5, label="Trasa A*")
        self.marker_slam, = self.ax_slam.plot([0], [0], "o", color="#ef476f", markersize=10, label="Estymacja")
        self.marker_goal_slam, = self.ax_slam.plot([], [], "X", color="#ffd166", markersize=12)
        self.ax_slam.legend(loc="upper right", fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas.mpl_connect("button_press_event", self._on_mouse_click)
        self.canvas.mpl_connect("motion_notify_event", self._on_mouse_drag)

    def _on_mouse_click(self, event: Any) -> None:
        if event.inaxes not in (self.ax_sim, self.ax_slam) or event.xdata is None or event.ydata is None:
            return
        mode = self.mode.get()
        if mode == "WALL" and event.inaxes == self.ax_sim:
            self.env.add_brush_stroke(event.xdata, event.ydata, radius_m=0.10)
            self.img_env.set_data(self.env.grid)
            self.canvas.draw_idle()
        elif mode == "SET_ROBOT":
            new_pos = np.array([event.xdata, event.ydata, 0.0], dtype=np.float64)
            self.engine.reset_pose(new_pos)
            self.gt_history = [(event.xdata, event.ydata)]
            self.marker_gt.set_data([new_pos[0]], [new_pos[1]])
            self.marker_slam.set_data([new_pos[0]], [new_pos[1]])
            self.canvas.draw_idle()
        elif mode == "SET_GOAL":
            self.engine.goal_world = np.array([event.xdata, event.ydata], dtype=np.float64)
            self.marker_goal_sim.set_data([event.xdata], [event.ydata])
            self.marker_goal_slam.set_data([event.xdata], [event.ydata])
            self.canvas.draw_idle()

    def _on_mouse_drag(self, event: Any) -> None:
        if event.button == 1 and event.inaxes == self.ax_sim and self.mode.get() == "WALL":
            if event.xdata is not None and event.ydata is not None:
                self.env.add_brush_stroke(event.xdata, event.ydata, radius_m=0.10)
                self.img_env.set_data(self.env.grid)
                self.canvas.draw_idle()

    def toggle_navigation(self) -> None:
        self.engine.nav_enabled = not self.engine.nav_enabled
        self.btn_nav.config(text="⏸ ZATRZYMAJ" if self.engine.nav_enabled else "🚀 JEDŹ DO CELU")

    def clear_walls(self) -> None:
        self.env.clear()
        self.img_env.set_data(self.env.grid)
        self.canvas.draw_idle()

    def reset_map(self) -> None:
        self.engine.reset_pose(self.engine.true_pose)
        self.gt_history = []
        self.canvas.draw_idle()

    def _poll_simulation(self) -> None:
        has_update = False
        try:
            while not self.data_queue.empty():
                snap = self.data_queue.get_nowait()
                self.img_slam.set_data(snap["map_image"])
                sc = snap["scan_world"]
                if sc.shape[0] > 0:
                    self.scatter_scan.set_data(sc[:, 0], sc[:, 1])
                self.marker_slam.set_data([snap["pose"][0]], [snap["pose"][1]])

                tp = snap["true_pose"]
                self.marker_gt.set_data([tp[0]], [tp[1]])
                self.gt_history.append((tp[0], tp[1]))
                gx, gy = zip(*self.gt_history)
                self.line_gt.set_data(gx, gy)

                if snap["path_pts"]:
                    px, py = zip(*snap["path_pts"])
                    self.line_path.set_data(px, py)
                else:
                    self.line_path.set_data([], [])

                err = math.hypot(snap["pose"][0] - tp[0], snap["pose"][1] - tp[1])
                crash = " | 💥 CRASH!" if snap["crashed"] else ""
                self.lbl_status.config(text=f"{snap['state_msg']}{crash} | Błąd: {err:.3f}m | RMSE: {snap['rmse']:.3f}m")
                has_update = True
        except queue.Empty:
            pass

        if has_update:
            self.canvas.draw_idle()
        self.root.after(40, self._poll_simulation)


if __name__ == "__main__":
    app_root = tk.Tk()
    InteractiveSimulatorApp(app_root)
    app_root.mainloop()