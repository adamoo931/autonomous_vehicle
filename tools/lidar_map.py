#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Autonomous Vehicle Navigation Brain – Enhanced SLAM + Stop-and-Go Navigation
with Proximity Guard, Virtual Wall Detection, and Robust ICP with Quality Gating.

Cechy:
- Stabilny SLAM w World Frame (CSM + ICP z ograniczonym kątem poszukiwania)
- Gating jakości: RMSE <= 0.085m, inlier ratio >= 0.35
- Wirtualne ściany z czujników krawędzi (CNY70)
- Dylatacja przeszkód (strefa 10cm bezpieczeństwa)
- Optymistyczny A* z karą za nieznane obszary
- Stop-and-Go z Proximity Guard (min 20cm / 0.2m do przeszkód)
"""

import json
import math
import queue
import threading
import time
import urllib.request
import urllib.error
from typing import Any, Dict, List, Optional, Sequence, Tuple

import tkinter as tk
from tkinter import ttk

import numpy as np
from scipy.linalg import svd
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree

# ============================================================================
# KONFIGURACJA PARAMETRÓW
# ============================================================================
DEFAULT_HOST = "192.168.137.51"  # IP modułu ESP32
CELL_SIZE_M = 0.05               # 5 cm na komórkę
GRID_WIDTH = 401                 # 401 x 0.05m = 20.05 m
GRID_HEIGHT = 401
MAX_RANGE_M = 5.5                # Zasięg LiDARu
MIN_RANGE_M = 0.08               # Martwa strefa

HTTP_TIMEOUT_S = 0.8
UI_REFRESH_MS = 40

# Progi ICP i keyframes
ICP_MAX_CORR_DIST = 0.22         # Promień korespondencji [m]
ICP_MAX_RMSE = 0.085             # Max RMSE dla akceptacji
ICP_MIN_INLIER_RATIO = 0.35      # Min odsetek inlierów
ICP_MIN_INLIERS = 20
KEYFRAME_MIN_DIST = 0.08         # Min trans. do keyframe'a
KEYFRAME_MIN_ANGLE = math.radians(6.0)

# Bezpieczeństwo i dylatacja
ROBOT_SAFETY_RADIUS_CELLS = 2    # 10 cm bufora
PROXIMITY_GUARD_M = 0.20         # 20 cm próg minimalnej odległości
PROXIMITY_GUARD_ANGLE = math.radians(35)  # Stożek przodu (±35°)

# Kolory wizualizacji
COLOR_ROBOT = "#ef476f"
COLOR_PATH = "#ffd166"
COLOR_SCAN = "#06d6a0"
COLOR_GOAL = "#ffb703"

# ============================================================================
# ALGORYTMY GEOMETRYCZNE
# ============================================================================
def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Generuje listę komórek na odcinku (x0, y0) -> (x1, y1)."""
    points: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
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


# ============================================================================
# PROBABILISTIC MAP MANAGER
# ============================================================================
class ProbabilisticMapManager:
    """
    Occupancy Grid (401x401 cells, cell_size=0.05m).
    -1: Unknown (exploration), 0: Free space, 100: Obstacles (walls)
    """
    def __init__(self, width: int, height: int, cell_size: float) -> None:
        self.width = width
        self.height = height
        self.cell_size = cell_size
        # -1: unknown, 0: free, 100: obstacle
        self.raw_grid = np.full((height, width), -1, dtype=np.int8)
        self.origin_x = width // 2
        self.origin_y = height // 2
        self.inflation_struct = np.ones(
            (2 * ROBOT_SAFETY_RADIUS_CELLS + 1, 2 * ROBOT_SAFETY_RADIUS_CELLS + 1), dtype=bool
        )

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        ix = int(math.floor(x / self.cell_size)) + self.origin_x
        iy = int(math.floor(y / self.cell_size)) + self.origin_y
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates."""
        x = (ix - self.origin_x + 0.5) * self.cell_size
        y = (iy - self.origin_y + 0.5) * self.cell_size
        return x, y

    def is_in_bounds(self, ix: int, iy: int) -> bool:
        """Check if grid index is within bounds."""
        return 0 <= ix < self.width and 0 <= iy < self.height

    def update_from_scan(self, scan_points: np.ndarray, pose: np.ndarray,
                        edge_detected: bool = False) -> None:
        """
        Update map with new scan using Bresenham raycasting.
        - scan_points: N x 2 array of (x, y) in robot frame
        - pose: [x, y, theta] in world frame
        - edge_detected: If True, insert virtual wall only forward, away from robot body.
        """
        rx, ry = self.world_to_grid(pose[0], pose[1])
        c_th = math.cos(pose[2])
        s_th = math.sin(pose[2])

        # Ustaw wolną przestrzeń wzdłuż promieni + zaznacz przeszkody
        for px, py in scan_points:
            dist = math.hypot(px, py)
            if dist < MIN_RANGE_M or dist > MAX_RANGE_M:
                continue
            if dist < 0.12:
                continue

            # Transform to world frame
            wx = pose[0] + c_th * px - s_th * py
            wy = pose[1] + s_th * px + c_th * py
            gx, gy = self.world_to_grid(wx, wy)

            if self.is_in_bounds(gx, gy):
                # Raycasting: free space from robot to hit point
                line = bresenham_line(rx, ry, gx, gy)
                for lx, ly in line[:-1]:
                    if self.is_in_bounds(lx, ly) and (lx, ly) != (rx, ry) and self.raw_grid[ly, lx] != 100:
                        self.raw_grid[ly, lx] = 0
                # Mark hit point as obstacle
                if (gx, gy) != (rx, ry):
                    self.raw_grid[gy, gx] = 100

        # VIRTUAL WALL: tylko na przodzie, co najmniej 18 cm i bez pokrycia z robotem
        if edge_detected:
            wall_dist_m = 0.18
            wall_x = pose[0] + math.cos(pose[2]) * wall_dist_m
            wall_y = pose[1] + math.sin(pose[2]) * wall_dist_m
            wx, wy = self.world_to_grid(wall_x, wall_y)
            if self.is_in_bounds(wx, wy) and (wx, wy) != (rx, ry):
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        nx, ny = wx + dx, wy + dy
                        if self.is_in_bounds(nx, ny) and (nx, ny) != (rx, ry):
                            self.raw_grid[ny, nx] = 100

    def get_occupied_points(self, max_points: int = 3000) -> np.ndarray:
        """Get random sample of obstacle points for ICP."""
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
        """Generate dilated obstacle mask for safe path planning."""
        occupied = (self.raw_grid == 100)
        return binary_dilation(occupied, structure=self.inflation_struct, border_value=0)

    def to_image(self) -> np.ndarray:
        """Convert grid to RGB image for visualization."""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[self.raw_grid == -1] = (16, 20, 26)     # Unknown (dark blue-gray)
        img[self.raw_grid == 0] = (220, 226, 220)   # Free (light gray)
        img[self.raw_grid == 100] = (15, 15, 15)    # Obstacles (black)
        return img


# ============================================================================
# ROBUST LOCALIZER – CSM + ICP z QUALITY GATING
# ============================================================================
class RobustLocalizer:
    """
    Two-stage localization: Correlative Search + Precise ICP.
    With quality gating: RMSE <= 0.085m AND inlier_ratio >= 0.35.
    """
    def __init__(self, max_iter: int = 10, tol: float = 1e-4) -> None:
        self.max_iter = max_iter
        self.tol = tol

    @staticmethod
    def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """Transform points from robot frame to world frame using pose."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        return (points @ rot.T) + pose[:2]

    def coarse_angle_search(self,
                            scan_pts: np.ndarray,
                            raw_grid: np.ndarray,
                            origin_xy: Tuple[int, int],
                            cell_size: float,
                            base_pose: np.ndarray,
                            step_deg: float = 2.0) -> float:
        """
        Correlative angle search within ±30° window of current orientation
        (instead of full 360°) to eliminate 180° flips in narrow corridors.
        """
        # Ograniczony zakres: ±30° wokół bieżącej orientacji
        center_deg = math.degrees(base_pose[2])
        angles_deg = np.arange(center_deg - 30, center_deg + 31, step_deg)
        angles = np.radians(angles_deg)
        
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

    def localize(self,
                 scan_pts: np.ndarray,
                 map_pts: np.ndarray,
                 raw_grid: np.ndarray,
                 origin_xy: Tuple[int, int],
                 cell_size: float,
                 current_pose: np.ndarray) -> Tuple[np.ndarray, bool, float, float]:
        """
        Localize robot using scan and map with quality gating.
        Returns: (pose, is_valid, rmse, inlier_ratio)
        """
        if len(scan_pts) < 15 or len(map_pts) < 15:
            return current_pose.copy(), False, 999.0, 0.0

        # KROK 1: Correlative Angle Search (±30° window)
        best_th = self.coarse_angle_search(scan_pts, raw_grid, origin_xy, cell_size, current_pose)
        pose = np.array([current_pose[0], current_pose[1], best_th], dtype=np.float64)

        tree = cKDTree(map_pts)
        final_rmse = 999.0
        inliers_count = 0

        # KROK 2: Precise ICP on SVD
        for iteration in range(self.max_iter):
            transformed = self.transform_points(scan_pts, pose)
            dists, idxs = tree.query(transformed, k=1, distance_upper_bound=ICP_MAX_CORR_DIST)
            valid = dists < ICP_MAX_CORR_DIST
            inliers_count = int(np.count_nonzero(valid))

            if inliers_count < ICP_MIN_INLIERS:
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

        # QUALITY GATING: Accept only if RMSE <= 0.085m AND inlier_ratio >= 0.35
        is_valid = (final_rmse <= ICP_MAX_RMSE and
                    inlier_ratio >= ICP_MIN_INLIER_RATIO and
                    inliers_count >= ICP_MIN_INLIERS)

        return (pose if is_valid else current_pose.copy()), is_valid, final_rmse, inlier_ratio


# ============================================================================
# OPTIMISTIC PATH PLANNER – A* z EXPLORATION WEIGHTS
# ============================================================================
class OptimisticPathPlanner:
    """
    A* pathfinding with exploration support.
    Cell costs: free=1.0, unknown=1.4, obstacles/inflated=infinity.
    Allows pathfinding through unexplored areas to reach goal.
    """
    def __init__(self, map_manager: ProbabilisticMapManager) -> None:
        self.map_mgr = map_manager

    @staticmethod
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Plan path from start to goal using A*.
        Returns list of grid cells from start to goal.
        """
        if not self.map_mgr.is_in_bounds(start[0], start[1]) or \
           not self.map_mgr.is_in_bounds(goal[0], goal[1]):
            return []

        costmap_inflated = self.map_mgr.get_inflated_costmap()

        # Jeśli cel jest w ścianie, znajdź bliski wolny spot
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

            # 8-directional movement
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = curr[0] + dx, curr[1] + dy
                if not self.map_mgr.is_in_bounds(nx, ny):
                    continue
                if costmap_inflated[ny, nx]:  # Obstacle or inflated zone
                    continue

                # Cost calculation
                step_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                cell_val = self.map_mgr.raw_grid[ny, nx]

                # Exploration bonus: unknown cells have higher cost but are traversable
                # 0 (free) -> 1.0, -1 (unknown) -> 1.4
                unknown_penalty = 1.4 if cell_val == -1 else 1.0
                tentative = g_score[curr] + step_cost * unknown_penalty

                if tentative < g_score.get((nx, ny), float("inf")):
                    came_from[(nx, ny)] = curr
                    g_score[(nx, ny)] = tentative
                    f_score = tentative + self.heuristic((nx, ny), goal)
                    open_set.append((f_score, (nx, ny)))

        return []


# ============================================================================
# AUTONOMOUS WORKER – STOP-AND-GO NAVIGATION z SLAM
# ============================================================================
class AutonomousWorker(threading.Thread):
    """
    Main navigation loop: full scan -> localization -> map update -> path planning -> Stop-and-Go step.
    Includes Proximity Guard: check front arc for obstacles >= 20cm.
    """
    def __init__(self, host: str, map_manager: ProbabilisticMapManager, out_queue: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.map_mgr = map_manager
        self.planner = OptimisticPathPlanner(self.map_mgr)
        self.out_queue = out_queue
        self.running = True

        self.localizer = RobustLocalizer()
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.last_keyframe_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.bootstrapped = False
        self.last_scan_world = np.empty((0, 2), dtype=np.float64)

        self.goal_world: Optional[np.ndarray] = None
        self.nav_enabled = False
        self.path_cells: List[Tuple[int, int]] = []
        self.state_msg = "IDLE"

    def stop(self) -> None:
        self.running = False

    def set_goal(self, goal_xy: Optional[np.ndarray], start_navigation: bool = False) -> None:
        self.goal_world = goal_xy
        self.nav_enabled = start_navigation

    def _http_get(self, path: str) -> Optional[Dict[str, Any]]:
        """GET request to ESP32."""
        url = f"http://{self.host}{path}"
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            return None

    def _http_post(self, path: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """POST request to ESP32."""
        url = f"http://{self.host}{path}"
        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(url, data=data,
                                         headers={"Content-Type": "application/json"},
                                         method="POST")
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception:
            return None

    def _check_proximity_guard(self, scan_local_pts: np.ndarray) -> Tuple[bool, float]:
        """
        Proximity Guard w układzie robota: sprawdzamy tylko punkty z przodu
        (|atan2(y,x)| <= 35°), a nie punkty w układzie świata.
        """
        if len(scan_local_pts) == 0:
            return True, 10.0

        min_dist = 10.0
        for px, py in scan_local_pts:
            dist = math.hypot(px, py)
            if dist == 0:
                continue
            angle_rad = math.atan2(py, px)
            if abs(angle_rad) <= PROXIMITY_GUARD_ANGLE:
                if dist < min_dist:
                    min_dist = dist
                if dist < PROXIMITY_GUARD_M:
                    return False, dist

        return True, min_dist

    def run(self) -> None:
        """Main navigation loop, sequenced as STOP -> SCAN -> PLAN -> EXECUTE -> SETTLE."""
        while self.running:
            if self.goal_world is None or not self.nav_enabled:
                time.sleep(0.08)
                continue

            # 1. STOP & SCAN
            data = self._http_get("/api/lidar/scan?since=0")
            if not data or "pts" not in data:
                time.sleep(0.1)
                continue

            pts_raw = data.get("pts", [])
            edge_detected = bool(data.get("edge_detected", False))
            collision = bool(data.get("collision", False))
            if len(pts_raw) < 20:
                time.sleep(0.08)
                continue

            arr = np.asarray(pts_raw, dtype=np.float64).reshape(-1, 2)
            angles = -(arr[:, 0] / 100.0 * math.pi / 180.0)
            dists = arr[:, 1] / 1000.0
            valid_mask = (dists >= MIN_RANGE_M) & (dists <= MAX_RANGE_M)
            if np.count_nonzero(valid_mask) < 15:
                time.sleep(0.08)
                continue

            angles = angles[valid_mask]
            dists = dists[valid_mask]
            scan_pts = np.vstack((np.cos(angles) * dists, np.sin(angles) * dists)).T

            map_pts = self.map_mgr.get_occupied_points()
            if not self.bootstrapped or len(map_pts) < 30:
                self.map_mgr.update_from_scan(scan_pts, self.pose, edge_detected)
                self.last_keyframe_pose = self.pose.copy()
                self.bootstrapped = True
                is_valid, rmse, ratio = True, 0.0, 1.0
                self.state_msg = "BOOTSTRAPPING..."
            else:
                origin = (self.map_mgr.origin_x, self.map_mgr.origin_y)
                new_pose, is_valid, rmse, ratio = self.localizer.localize(
                    scan_pts, map_pts, self.map_mgr.raw_grid, origin,
                    self.map_mgr.cell_size, self.pose
                )
                if is_valid:
                    self.pose = new_pose
                    d_trans = np.linalg.norm(self.pose[:2] - self.last_keyframe_pose[:2])
                    d_rot = abs((self.pose[2] - self.last_keyframe_pose[2] + math.pi) % (2 * math.pi) - math.pi)
                    if d_trans >= KEYFRAME_MIN_DIST or d_rot >= KEYFRAME_MIN_ANGLE:
                        self.map_mgr.update_from_scan(scan_pts, self.pose, edge_detected)
                        self.last_keyframe_pose = self.pose.copy()
                    self.state_msg = "SLAM OK"
                else:
                    self.state_msg = f"ICP REJECTED (RMSE={rmse:.3f}m, ratio={ratio:.2f})"

            self.last_scan_world = self.localizer.transform_points(scan_pts, self.pose)

            start_c = self.map_mgr.world_to_grid(self.pose[0], self.pose[1])
            goal_c = self.map_mgr.world_to_grid(self.goal_world[0], self.goal_world[1])
            self.path_cells = self.planner.plan(start_c, goal_c)

            if self.goal_world is not None:
                dist_to_goal = float(np.linalg.norm(self.pose[:2] - self.goal_world))
                if dist_to_goal < 0.25:
                    self.state_msg = "GOAL REACHED!"
                    self.nav_enabled = False
                    self.path_cells = []
                    time.sleep(0.08)
                    continue

            # 2. PLAN
            if len(self.path_cells) < 2:
                self.state_msg = "NO PATH AVAILABLE"
                time.sleep(0.12)
                continue

            target_cell = self.path_cells[min(6, len(self.path_cells) - 1)]
            target_wx, target_wy = self.map_mgr.grid_to_world(*target_cell)
            target_angle = math.atan2(target_wy - self.pose[1], target_wx - self.pose[0])
            angle_err = (target_angle - self.pose[2] + math.pi) % (2 * math.pi) - math.pi

            # 3. EXECUTE STEP
            is_clear, min_dist = self._check_proximity_guard(scan_pts)
            if not is_clear:
                self.state_msg = f"PROXIMITY GUARD: dist={min_dist:.2f}m < {PROXIMITY_GUARD_M}m"
                payload = {"pwm_l": -30, "pwm_r": -30, "duration_ms": 220}
                self._http_post("/api/autonomy/remote_move", payload)
                time.sleep(0.22 + 0.1)
                continue

            if abs(angle_err) > math.radians(15.0):
                turn_dir = 1 if angle_err > 0 else -1
                payload = {"pwm_l": -turn_dir * 30, "pwm_r": turn_dir * 30, "duration_ms": 180}
                self._http_post("/api/autonomy/remote_move", payload)
                self.state_msg = f"ROTATING: {math.degrees(angle_err):.0f}°"
            else:
                payload = {"pwm_l": 35, "pwm_r": 35, "duration_ms": 280}
                self._http_post("/api/autonomy/remote_move", payload)
                self.state_msg = f"STEP FORWARD: dist_to_goal={float(np.linalg.norm(self.pose[:2] - self.goal_world)):.2f}m"

            # 4. SETTLE
            time.sleep(0.10)

            path_pts = [self.map_mgr.grid_to_world(*c) for c in self.path_cells]
            snapshot = {
                "pose": self.pose.copy(),
                "map_image": self.map_mgr.to_image(),
                "scan_world": self.last_scan_world,
                "path_pts": path_pts,
                "goal_world": self.goal_world.copy() if self.goal_world is not None else None,
                "is_valid": is_valid,
                "rmse": rmse,
                "inlier_ratio": ratio,
                "state_msg": self.state_msg,
                "edge_detected": edge_detected,
                "collision": collision,
            }

            if self.out_queue.full():
                try:
                    self.out_queue.get_nowait()
                except queue.Empty:
                    pass
            self.out_queue.put_nowait(snapshot)

            time.sleep(0.06)


# ============================================================================
# GRAPHICAL USER INTERFACE – TKINTER + MATPLOTLIB
# ============================================================================
class NavigationApp:
    """Main GUI application for SLAM visualization and navigation control."""
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Autonomous Vehicle – Enhanced SLAM Navigation")
        self.root.geometry("1200x900")

        self.map_mgr = ProbabilisticMapManager(GRID_WIDTH, GRID_HEIGHT, CELL_SIZE_M)
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        self.data_queue: queue.Queue = queue.Queue(maxsize=2)
        self.worker: Optional[AutonomousWorker] = None
        self.goal_world: Optional[np.ndarray] = None

        self._build_ui()
        self.root.after(UI_REFRESH_MS, self._poll_queue)

    def _build_ui(self) -> None:
        """Build user interface."""
        ctrl = ttk.Frame(self.root, padding=8)
        ctrl.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(ctrl, text="ESP32 Host:").pack(side=tk.LEFT, padx=4)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(ctrl, textvariable=self.host_var, width=18).pack(side=tk.LEFT, padx=4)

        self.btn_toggle = ttk.Button(ctrl, text="▶ START SLAM", command=self.toggle_scan)
        self.btn_toggle.pack(side=tk.LEFT, padx=6)

        self.btn_nav = ttk.Button(ctrl, text="🚀 START NAVIGATION", command=self.toggle_nav, state=tk.DISABLED)
        self.btn_nav.pack(side=tk.LEFT, padx=4)

        ttk.Button(ctrl, text="CLEAR MAP", command=self.clear_map).pack(side=tk.LEFT, padx=4)

        self.lbl_status = ttk.Label(ctrl, text="Status: Ready", font=("Consolas", 9))
        self.lbl_status.pack(side=tk.LEFT, padx=12)

        # Matplotlib canvas
        import matplotlib
        matplotlib.use("TkAgg")
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure

        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-6.0, 6.0)
        self.ax.set_ylim(-6.0, 6.0)
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_title("Global Occupancy Map – SLAM with Stop-and-Go Navigation")
        self.ax.grid(True, linestyle=":", alpha=0.4)

        dummy_img = self.map_mgr.to_image()
        self.map_img = self.ax.imshow(
            dummy_img,
            extent=[-GRID_WIDTH // 2 * CELL_SIZE_M, GRID_WIDTH // 2 * CELL_SIZE_M,
                    -GRID_HEIGHT // 2 * CELL_SIZE_M, GRID_HEIGHT // 2 * CELL_SIZE_M],
            origin="lower"
        )
        self.scatter_scan, = self.ax.plot([], [], "o", color=COLOR_SCAN, markersize=2, alpha=0.6, label="LiDAR scan")
        self.path_line, = self.ax.plot([], [], color=COLOR_PATH, linewidth=2.5, label="Path (A*)")
        self.marker_robot, = self.ax.plot([0], [0], marker="o", color=COLOR_ROBOT, markersize=10, label="Robot")
        self.line_dir, = self.ax.plot([0, 0.25], [0, 0], color=COLOR_ROBOT, linewidth=2.5)
        self.marker_goal, = self.ax.plot([], [], marker="X", color=COLOR_GOAL, markersize=12, label="Goal")

        self.ax.legend(loc="upper right", fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self._on_map_click)

    def _on_map_click(self, event: Any) -> None:
        """Handle map click to set goal."""
        if event.inaxes != self.ax:
            return
        self.goal_world = np.array([event.xdata, event.ydata], dtype=np.float64)
        self.marker_goal.set_data([self.goal_world[0]], [self.goal_world[1]])
        self.btn_nav.config(state=tk.NORMAL)
        if self.worker:
            self.worker.set_goal(self.goal_world, start_navigation=False)
        self.canvas.draw_idle()

    def toggle_nav(self) -> None:
        """Start navigation to goal."""
        if self.worker and self.goal_world is not None:
            self.worker.set_goal(self.goal_world, start_navigation=True)
            self.lbl_status.config(text="Status: Navigation started...")

    def toggle_scan(self) -> None:
        """Start/stop SLAM."""
        if self.worker and self.worker.is_alive():
            self.worker.stop()
            self.worker = None
            self.btn_toggle.config(text="▶ START SLAM")
            self.btn_nav.config(state=tk.DISABLED)
            self.lbl_status.config(text="Status: Stopped")
        else:
            host = self.host_var.get().strip()
            self.worker = AutonomousWorker(host, self.map_mgr, self.data_queue)
            if self.goal_world is not None:
                self.worker.set_goal(self.goal_world, start_navigation=False)
            self.worker.start()
            self.btn_toggle.config(text="⏸ STOP")

    def clear_map(self) -> None:
        """Clear map and reset."""
        if self.worker and self.worker.is_alive():
            self.worker.stop()
            self.worker = None
            self.btn_toggle.config(text="▶ START SLAM")
        self.map_mgr = ProbabilisticMapManager(GRID_WIDTH, GRID_HEIGHT, CELL_SIZE_M)
        self.pose[:] = 0.0
        self.goal_world = None
        self.btn_nav.config(state=tk.DISABLED)
        self.map_img.set_data(self.map_mgr.to_image())
        self.scatter_scan.set_data([], [])
        self.path_line.set_data([], [])
        self.marker_goal.set_data([], [])
        self.marker_robot.set_data([0], [0])
        self.line_dir.set_data([0, 0.25], [0, 0])
        self.canvas.draw_idle()
        self.lbl_status.config(text="Status: Map cleared")

    def _poll_queue(self) -> None:
        """Poll worker queue for updates."""
        try:
            while not self.data_queue.empty():
                snap = self.data_queue.get_nowait()
                self.pose = snap["pose"]
                self.map_img.set_data(snap["map_image"])

                sc_pts = snap["scan_world"]
                if sc_pts.shape[0] > 0:
                    self.scatter_scan.set_data(sc_pts[:, 0], sc_pts[:, 1])

                self.marker_robot.set_data([self.pose[0]], [self.pose[1]])
                self.line_dir.set_data(
                    [self.pose[0], self.pose[0] + math.cos(self.pose[2]) * 0.4],
                    [self.pose[1], self.pose[1] + math.sin(self.pose[2]) * 0.4]
                )

                path = snap["path_pts"]
                if path:
                    xs, ys = zip(*path)
                    self.path_line.set_data(xs, ys)
                else:
                    self.path_line.set_data([], [])

                deg = math.degrees(self.pose[2])
                edge_msg = "🔴 EDGE" if snap.get("edge_detected", False) else ""
                collision_msg = "💥 COLLISION" if snap.get("collision", False) else ""
                extra = f" {edge_msg} {collision_msg}".strip()

                self.lbl_status.config(
                    text=f"{snap['state_msg']}{extra} | X:{self.pose[0]:+.2f}m Y:{self.pose[1]:+.2f}m "
                         f"θ:{deg:.0f}° | RMSE:{snap['rmse']:.3f}m ratio:{snap['inlier_ratio']:.2f}"
                )
                self.canvas.draw_idle()
        except queue.Empty:
            pass

        self.root.after(UI_REFRESH_MS, self._poll_queue)


if __name__ == "__main__":
    root = tk.Tk()
    app = NavigationApp(root)
    root.mainloop()
