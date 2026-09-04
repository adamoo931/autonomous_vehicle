#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stop-Scan-and-Go: pętla integracyjna łącząca ESP32 (ruch/LIDAR) z SLAM+A*
z lidar_map.py.

Dlaczego nie wystarczy "wyślij ruch, time.sleep(duration_ms), skanuj"
-----------------------------------------------------------------------
1. autonomy_execute_remote_move() na ESP32 jest NIEBLOKUJĄCE - handler HTTP
   odpowiada natychmiast po ustawieniu stanu ST_REMOTE_EXEC, fizyczny ruch
   wykonuje się dopiero w kolejnych cyklach pętli sterowania. Odpowiedź HTTP
   200 OK NIE oznacza "ruch zakończony".
2. Jeśli wyślesz kolejną komendę ruchu zanim poprzedni ST_REMOTE_EXEC się
   zakończy, autonomy_execute_remote_move() zwraca false i ruch jest
   ignorowany (patrz `if (s_state == ST_REMOTE_EXEC) return false;`) - przy
   samym time.sleep(duration_ms) opóźnienia Wi-Fi (transmisja, kolejkowanie,
   GC w lwIP) łatwo sprawiają, że budzisz się too early i tracisz krok.
3. Odwrotnie - zbyt długi bufor "na sztywno" marnuje czas na każdym kroku
   i przy dłuższej trasie się sumuje.
Rozwiązanie: po wysłaniu ruchu POLLUJEMY /api/sensors i czekamy, aż pole
autonomy.state opuści "Remote Exec"/"Remote Wait", z twardym timeoutem
(duration_ms + margines na latencję Wi-Fi) jako siatką bezpieczeństwa.
"""

from __future__ import annotations

import json
import logging
import math
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

# Reużywamy istniejącego stosu SLAM/A* zamiast go duplikować.
from lidar_map import (
    MAX_RANGE_M,
    MIN_RANGE_M,
    KEYFRAME_MIN_ANGLE,
    KEYFRAME_MIN_DIST,
    OptimisticPathPlanner,
    ProbabilisticMapManager,
    RobustLocalizer,
)

log = logging.getLogger("stop_scan_go")

# ============================================================================
# KONFIGURACJA
# ============================================================================
MOVE_LATENCY_MARGIN_S = 0.35     # bufor na opóźnienie Wi-Fi ponad duration_ms
MOVE_POLL_INTERVAL_S = 0.05      # jak często odpytujemy /api/sensors podczas ruchu
MOVE_HARD_TIMEOUT_S = 4.0        # absolutny sufit - dłużej nie czekamy na 1 krok

HTTP_CONNECT_TIMEOUT_S = 0.6     # timeout pojedynczej próby GET/scan
HTTP_MOVE_TIMEOUT_S = 0.6        # timeout pojedynczej próby POST (sama odp. handlera jest szybka)
HTTP_MAX_RETRIES = 4

# --- Moc silników dla remote_move -----------------------------------------
# WAŻNE: stan ST_REMOTE_EXEC w autonomy.c (case ST_REMOTE_EXEC) ustawia PWM
# BEZPOŚREDNIO przez motor_set_left/right(), bez impulsu rozruchowego
# (KICK_POWER=45%/KICK_MS=150ms), który normalny tryb autonomiczny dostaje
# w ST_START/ST_CRUISE. Innymi słowy: krok Stop-Scan-and-Go zawsze rusza
# z zera i za każdym razem musi sam przełamać tarcie statyczne - bez pomocy
# impulsu, który reszta kodu ma wbudowaną.
#
# Wartości niżej dobrane pod ten brak, zgodnie z tym, co już empirycznie
# ustalono w autonomy.c dla tego podwozia:
#   - jazda na wprost (oba koła): udokumentowany próg ruchu to SP_CRUISE=35%,
#     ale to próg PRZY włączonym kicku - bez kicku bierzemy margines i
#     celujemy w KICK_POWER=45%, bo to wartość faktycznie sprawdzona jako
#     "łamie tarcie statyczne" na tym podwoziu.
#   - obrót w miejscu (jedno koło stoi, dodaje tarcie): projekt ma już
#     osobną, zmierzoną wartość ALIGN_TURN_POWER=38% właśnie dlatego, że
#     "25% okazało się za mało (pojazd nie ruszał / ledwo pełzał)" - patrz
#     komentarz w autonomy.c. Wcześniejsze 30% w tej pętli było NIŻSZE od
#     tego zmierzonego progu i nie miało żadnego uzasadnienia w kodzie C
#     (SP_TURN=30 jest tam zdefiniowane, ale nigdzie faktycznie nieużywane).
REMOTE_PWM_FORWARD = 45   # = KICK_POWER w autonomy.c
REMOTE_PWM_TURN = 38      # = ALIGN_TURN_POWER w autonomy.c
HTTP_BACKOFF_BASE_S = 0.12       # 0.12, 0.24, 0.48, 0.96 ... (exponential backoff)
HTTP_BACKOFF_MAX_S = 1.0

REMOTE_BUSY_STATES = {"Remote Exec", "Remote Wait"}


class RobotUnreachableError(RuntimeError):
    """Wyczerpano retry - ESP32 nie odpowiada (Wi-Fi padło / robot zawieszony)."""


# ============================================================================
# KLIENT HTTP - retry + backoff, osobne timeouty dla GET (skan) i POST (ruch)
# ============================================================================
class Esp32Client:
    def __init__(self, host: str) -> None:
        self.host = host

    def _request(
        self,
        method: str,
        path: str,
        payload: Optional[Dict[str, Any]] = None,
        timeout: float = HTTP_CONNECT_TIMEOUT_S,
        max_retries: int = HTTP_MAX_RETRIES,
    ) -> Dict[str, Any]:
        """
        Pojedyncze żądanie z exponential backoff. Rozróżniamy:
        - timeout / URLError (Wi-Fi latency, zgubiony pakiet)  -> retry
        - HTTPError 4xx/5xx (serwer odpowiedział, ale odmówił) -> retry też,
          bo na ESP32 zdarza się chwilowe zajęcie httpd przez inny handler
        Po wyczerpaniu prób podnosimy RobotUnreachableError - wołający MUSI
        to obsłużyć (nie zakładamy, że robot "na pewno gdzieś jest").
        """
        url = f"http://{self.host}{path}"
        data = json.dumps(payload).encode("utf-8") if payload is not None else None
        headers = {"Content-Type": "application/json"} if payload is not None else {}

        last_exc: Optional[Exception] = None
        for attempt in range(max_retries):
            try:
                req = urllib.request.Request(url, data=data, headers=headers, method=method)
                with urllib.request.urlopen(req, timeout=timeout) as resp:
                    return json.loads(resp.read().decode("utf-8"))
            except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError,
                    json.JSONDecodeError, ConnectionError) as exc:
                last_exc = exc
                backoff = min(HTTP_BACKOFF_BASE_S * (2 ** attempt), HTTP_BACKOFF_MAX_S)
                log.warning("HTTP %s %s failed (attempt %d/%d): %s - retry in %.2fs",
                            method, path, attempt + 1, max_retries, exc, backoff)
                time.sleep(backoff)

        raise RobotUnreachableError(f"{method} {path} failed after {max_retries} attempts: {last_exc}")

    def get_sensors(self) -> Dict[str, Any]:
        return self._request("GET", "/api/sensors", timeout=HTTP_CONNECT_TIMEOUT_S)

    def get_lidar_scan(self, since: int = 0) -> Dict[str, Any]:
        # Skan bywa większy niż zwykłe /api/sensors - dajemy mu własny, dłuższy timeout.
        return self._request("GET", f"/api/lidar/scan?since={since}", timeout=1.2)

    def remote_move(self, pwm_l: int, pwm_r: int, duration_ms: int) -> Dict[str, Any]:
        payload = {"pwm_l": pwm_l, "pwm_r": pwm_r, "duration_ms": duration_ms}
        return self._request("POST", "/api/autonomy/remote_move", payload, timeout=HTTP_MOVE_TIMEOUT_S)


# ============================================================================
# WYNIK JEDNEGO KROKU - do logiki wyższego poziomu (retry planowania itd.)
# ============================================================================
@dataclass
class StepResult:
    ok: bool
    collision: bool
    edge: bool
    scan_pts: np.ndarray          # N x 2 w układzie robota [m]
    pose: np.ndarray              # [x, y, theta] po lokalizacji
    is_localized: bool
    rmse: float
    inlier_ratio: float
    message: str


class StopScanGoController:
    """
    Jeden cykl = STOP (poprzedni krok już zakończony) -> WYŚLIJ RUCH -> CZEKAJ
    NA FIZYCZNE ZAKOŃCZENIE -> SKANUJ -> ZAKTUALIZUJ MAPĘ -> ZAPLANUJ (A*).
    """

    def __init__(self, host: str, map_mgr: ProbabilisticMapManager) -> None:
        self.client = Esp32Client(host)
        self.map_mgr = map_mgr
        self.localizer = RobustLocalizer()
        self.planner = OptimisticPathPlanner(map_mgr)

        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.last_keyframe_pose = self.pose.copy()
        self.bootstrapped = False

    # ------------------------------------------------------------------
    # Krok 1: wyślij ruch i poczekaj, aż FIZYCZNIE się zakończy
    # ------------------------------------------------------------------
    def _wait_for_move_completion(self, duration_ms: int) -> Tuple[bool, bool]:
        """
        Polluje /api/sensors dopóki autonomy.state nie opuści stanu
        "w trakcie ruchu". Zwraca (collision, edge) zaobserwowane po drodze.

        Deadline = duration_ms (czas ruchu zadeklarowany do ESP32) + margines
        na Wi-Fi. To NIE jest zwykły sleep: jeśli robot skończy szybciej (np.
        kolizja przerwała ruch), wracamy od razu; jeśli sieć się zatka,
        czekamy dłużej niż surowe duration_ms, zamiast zgadywać.
        """
        deadline = time.monotonic() + min(
            duration_ms / 1000.0 + MOVE_LATENCY_MARGIN_S, MOVE_HARD_TIMEOUT_S
        )
        collision = False
        edge = False
        consecutive_poll_failures = 0

        while time.monotonic() < deadline:
            try:
                data = self.client.get_sensors()
                consecutive_poll_failures = 0
            except RobotUnreachableError:
                # Wi-Fi się zacięło w trakcie oczekiwania - to NIE jest to
                # samo co "ruch trwa": nie wiemy, gdzie jest robot. Liczymy
                # nieudane odpytania i po kilku z rzędu przerywamy zamiast
                # czekać w nieskończoność na martwym łączu.
                consecutive_poll_failures += 1
                if consecutive_poll_failures >= 5:
                    log.error("Utracono łączność z ESP32 podczas oczekiwania na ruch.")
                    break
                continue

            auto = data.get("autonomy", {})
            collision = collision or bool(auto.get("remote_collision", False))
            edge = edge or bool(auto.get("remote_edge", False))
            state = auto.get("state", "")

            if state not in REMOTE_BUSY_STATES:
                return collision, edge  # ruch fizycznie zakończony

            time.sleep(MOVE_POLL_INTERVAL_S)

        log.warning("Timeout oczekiwania na zakończenie ruchu (duration_ms=%d) - "
                    "kontynuuję ostrożnie, traktując jako zakończony.", duration_ms)
        return collision, edge

    def execute_move(self, pwm_l: int, pwm_r: int, duration_ms: int) -> Tuple[bool, bool, bool]:
        """
        Wysyła ruch i czeka na jego fizyczne zakończenie.
        Zwraca (accepted, collision, edge). accepted=False oznacza, że ESP32
        odrzuciło komendę (poprzedni ruch jeszcze trwał) - wołający powinien
        odczekać chwilę i spróbować ponownie, NIE ignorować tego faktu.
        """
        try:
            resp = self.client.remote_move(pwm_l, pwm_r, duration_ms)
        except RobotUnreachableError as exc:
            log.error("Nie udało się wysłać komendy ruchu: %s", exc)
            return False, False, False

        accepted = bool(resp.get("ok", resp.get("collision") is not None))
        log.debug("ESP32 odpowiedziało na remote_move: %s (accepted=%s)", resp, accepted)
        if not accepted:
            log.info("ESP32 odrzuciło ruch (poprzedni jeszcze trwa) - czekam i ponawiam.")
            time.sleep(0.1)
            return False, False, False

        collision, edge = self._wait_for_move_completion(duration_ms)
        # Kolizja/edge mogły też przyjść bezpośrednio w odpowiedzi POST.
        collision = collision or bool(resp.get("collision", False))
        edge = edge or bool(resp.get("edge_detected", False))
        log.info("Ruch zakończony: collision=%s, edge=%s", collision, edge)
        return True, collision, edge

    # ------------------------------------------------------------------
    # Krok 2: skanuj i zaktualizuj SLAM (lokalizacja + mapa)
    # ------------------------------------------------------------------
    def scan_and_update(self, edge_detected: bool) -> StepResult:
        try:
            data = self.client.get_lidar_scan(since=0)
        except RobotUnreachableError as exc:
            return StepResult(False, False, edge_detected, np.empty((0, 2)),
                               self.pose, False, 999.0, 0.0, f"Skan nieudany: {exc}")

        pts_raw = data.get("pts", [])
        collision = bool(data.get("collision", False))
        if len(pts_raw) < 20:
            return StepResult(False, collision, edge_detected, np.empty((0, 2)),
                               self.pose, False, 999.0, 0.0, "Zbyt mało punktów skanu")

        arr = np.asarray(pts_raw, dtype=np.float64).reshape(-1, 2)
        angles = -(arr[:, 0] / 100.0 * math.pi / 180.0)
        dists = arr[:, 1] / 1000.0
        valid = (dists >= MIN_RANGE_M) & (dists <= MAX_RANGE_M)
        if np.count_nonzero(valid) < 15:
            return StepResult(False, collision, edge_detected, np.empty((0, 2)),
                               self.pose, False, 999.0, 0.0, "Zbyt mało poprawnych punktów")

        angles, dists = angles[valid], dists[valid]
        scan_pts = np.vstack((np.cos(angles) * dists, np.sin(angles) * dists)).T

        map_pts = self.map_mgr.get_occupied_points()
        if not self.bootstrapped or len(map_pts) < 30:
            self.map_mgr.update_from_scan(scan_pts, self.pose, edge_detected)
            self.last_keyframe_pose = self.pose.copy()
            self.bootstrapped = True
            return StepResult(True, collision, edge_detected, scan_pts, self.pose,
                               True, 0.0, 1.0, "BOOTSTRAPPING")

        origin = (self.map_mgr.origin_x, self.map_mgr.origin_y)
        new_pose, is_valid, rmse, ratio = self.localizer.localize(
            scan_pts, map_pts, self.map_mgr.raw_grid, origin,
            self.map_mgr.cell_size, self.pose
        )
        msg = "SLAM OK"
        if is_valid:
            self.pose = new_pose
            d_trans = np.linalg.norm(self.pose[:2] - self.last_keyframe_pose[:2])
            d_rot = abs((self.pose[2] - self.last_keyframe_pose[2] + math.pi) % (2 * math.pi) - math.pi)
            if d_trans >= KEYFRAME_MIN_DIST or d_rot >= KEYFRAME_MIN_ANGLE:
                self.map_mgr.update_from_scan(scan_pts, self.pose, edge_detected)
                self.last_keyframe_pose = self.pose.copy()
        else:
            msg = f"ICP REJECTED (RMSE={rmse:.3f}, ratio={ratio:.2f})"

        return StepResult(True, collision, edge_detected, scan_pts, self.pose,
                           is_valid, rmse, ratio, msg)

    # ------------------------------------------------------------------
    # BRAKUJĄCY ELEMENT: detekcja mety magnetycznej + źródła ciepła.
    # ------------------------------------------------------------------
    # Ta pętla wcześniej jechała tylko do zadanego punktu (x, y) na mapie i
    # na tym kończyła misję - nie sprawdzała w ogóle mety ani pirometru.
    # Tymczasem main.c na ESP32 i tak niezależnie: (1) wykrywa metę Hallem i
    # włącza tryb szukania pirometrem (pyrometer_start_search()), (2) po
    # wykryciu ciepła wywołuje motor_stop() z osobnego zadania sensorowego, a
    # (3) autonomy.c NIEZALEŻNIE od tego, czy silnikami steruje C czy Python
    # (nawet w trakcie ST_REMOTE_EXEC!), sam sprawdza próg ciepła i ustawia
    # s_enabled=false. Problem: Python o tym nic nie wiedział i po prostu
    # wysyłał kolejny remote_move w następnej iteracji, mogąc "odblokować"
    # jazdę zaraz po tym, jak firmware się zatrzymała. Ta metoda to naprawia -
    # pętla pyta ESP32 o stan przed KAŻDYM krokiem i przestaje jechać,
    # niezależnie od tego, kto fizycznie wykrył metę/ciepło jako pierwszy.
    FINISH_HEAT_DELTA_C = 1.5   # musi być spójne z FINISH_HEAT_DELTA_C w autonomy.c

    def check_finish(self) -> Tuple[bool, str]:
        """Zwraca (czy_zakonczono, powod) na podstawie /api/sensors."""
        try:
            data = self.client.get_sensors()
        except RobotUnreachableError:
            return False, ""

        auto = data.get("autonomy", {})
        if not auto.get("enabled", True):
            # s_enabled=false - albo meta/ciepło, albo ktoś zatrzymał ręcznie
            # z dashboardu. W obu przypadkach Python NIE powinien jechać dalej.
            return True, "ESP32 zgłasza autonomy.enabled=false"

        pyro = data.get("pyrometer", {})
        if pyro.get("hot_detected", False):
            return True, "pyrometer.hot_detected=true (tryb szukania po mecie)"

        obj = pyro.get("object_temp")
        amb = pyro.get("ambient_temp")
        if obj is not None and amb is not None and (obj - amb) >= self.FINISH_HEAT_DELTA_C:
            return True, f"delta ciepła {obj - amb:.1f}°C >= {self.FINISH_HEAT_DELTA_C}°C"

        return False, ""

    # ------------------------------------------------------------------
    # Pętla główna: Stop-Scan-and-Go do celu w układzie świata
    # ------------------------------------------------------------------
    def run_to_goal(self, goal_world: np.ndarray, goal_tolerance_m: float = 0.25,
                     max_steps: int = 2000) -> bool:
        for step_idx in range(max_steps):
            finished, reason = self.check_finish()
            if finished:
                log.info("Misja zakończona (krok %d): %s", step_idx, reason)
                return True

            dist_to_goal = float(np.linalg.norm(self.pose[:2] - goal_world))
            if dist_to_goal < goal_tolerance_m:
                log.info("Cel osiągnięty po %d krokach (bez detekcji ciepła - "
                         "sprawdź, czy to naprawdę koniec trasy).", step_idx)
                return True

            # 1) SKAN Z BIEŻĄCEJ POZYCJI (robot stoi po poprzednim kroku)
            result = self.scan_and_update(edge_detected=False)
            if not result.ok:
                log.warning("Krok %d: %s - ponawiam po krótkiej pauzie.", step_idx, result.message)
                time.sleep(0.1)
                continue

            # 2) PLANOWANIE (A*)
            start_c = self.map_mgr.world_to_grid(self.pose[0], self.pose[1])
            goal_c = self.map_mgr.world_to_grid(goal_world[0], goal_world[1])
            path = self.planner.plan(start_c, goal_c)
            if len(path) < 2:
                log.warning("Krok %d: brak ścieżki do celu.", step_idx)
                time.sleep(0.15)
                continue

            target_cell = path[min(6, len(path) - 1)]
            target_wx, target_wy = self.map_mgr.grid_to_world(*target_cell)
            target_angle = math.atan2(target_wy - self.pose[1], target_wx - self.pose[0])
            angle_err = (target_angle - self.pose[2] + math.pi) % (2 * math.pi) - math.pi

            # 3) WYKONAJ RUCH (retry aż zaakceptowany, z limitem prób)
            if abs(angle_err) > math.radians(15.0):
                turn = 1 if angle_err > 0 else -1
                pwm_l, pwm_r, duration_ms = -turn * REMOTE_PWM_TURN, turn * REMOTE_PWM_TURN, 180
            else:
                pwm_l, pwm_r, duration_ms = REMOTE_PWM_FORWARD, REMOTE_PWM_FORWARD, 280

            # Jawny log wyzwolenia ruchu - kluczowe przy debugowaniu "pojazd
            # nie jedzie": jeśli tego loga nie ma w konsoli podczas testu,
            # problem jest PRZED tym miejscem (nawigacja/cel nie włączone),
            # a nie w silnikach czy w komunikacji z ESP32.
            log.info("Krok %d: wyzwalam remote_move(pwm_l=%d, pwm_r=%d, duration_ms=%d) "
                     "[pose=(%.2f, %.2f, %.0f°), angle_err=%.0f°]",
                     step_idx, pwm_l, pwm_r, duration_ms,
                     self.pose[0], self.pose[1], math.degrees(self.pose[2]),
                     math.degrees(angle_err))

            for retry in range(3):
                accepted, collision, edge = self.execute_move(pwm_l, pwm_r, duration_ms)
                if accepted:
                    break
                log.info("Krok %d: ruch odrzucony, próba %d/3.", step_idx, retry + 1)
            else:
                log.error("Krok %d: nie udało się wykonać ruchu po 3 próbach - przerywam.", step_idx)
                return False

            if collision:
                log.warning("Krok %d: wykryto kolizję podczas ruchu - cofam się.", step_idx)
                self.execute_move(-30, -30, 220)

        log.warning("Osiągnięto max_steps=%d bez dotarcia do celu.", max_steps)
        return False


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    from lidar_map import CELL_SIZE_M, GRID_HEIGHT, GRID_WIDTH, DEFAULT_HOST

    mgr = ProbabilisticMapManager(GRID_WIDTH, GRID_HEIGHT, CELL_SIZE_M)
    controller = StopScanGoController(DEFAULT_HOST, mgr)
    controller.run_to_goal(np.array([2.0, 0.0]))
