#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_map.py – Desktopowy podgląd mapy przeszkód z lidaru robota.

Aplikacja łączy się z firmware ESP32 (tym samym, co serwuje dashboard WWW)
przez endpoint HTTP  GET /api/lidar/scan?since=N  i na bieżąco buduje mapę
przeszkód na wykresie punktowym.

Zasada mapowania (zgodnie z wymaganiem):
  • Pokazywane są TYLKO BIEŻĄCE przeszkody – każdy odczyt zastępuje poprzedni
    (najnowszy ~1 obrót lidaru). Dzięki temu obracający się/jadący robot NIE
    tworzy już skumulowanego miszmaszu – widać to, co lidar widzi teraz.
  • Przestrzeń jest dzielona na kwadratową siatkę o boku CELL (domyślnie 5 cm).
  • Każda zajęta komórka 5×5 cm = JEDNA czarna kropka na mapie.
  • Dzięki temu sześcian 5×5 cm to 1 kropka, a przedmiot 5×30 cm to 6 kropek
    (liczy się tylko szerokość i długość rzutowane na podłogę – wysokość jest
    nieistotna, bo lidar i tak skanuje w jednej płaszczyźnie).
  • Ściana = ciąg sąsiednich zajętych komórek = linia kropek.

Obsługa:
  • Wpisz adres IP robota (ten sam, pod którym działa dashboard), kliknij START.
  • Mapa pokazuje wyłącznie aktualny skan i odświeża się na bieżąco – idealne
    do weryfikacji, czy ściany/przeszkody rysują się poprawnie pod aktualnym
    położeniem i orientacją robota.
  • Suwaki/pola pozwalają dobrać rozmiar komórki, zasięg i orientację (offset kąta).

Wymagania:  Python 3.8+,  matplotlib,  numpy   (tkinter jest w standardzie).
    pip install -r requirements.txt
"""

import json
import math
import threading
import time
import urllib.request
from urllib.error import URLError

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk,
)

# ── Ustawienia domyślne ──────────────────────────────────────────────
DEFAULT_HOST       = "192.168.4.1"   # typowy adres ESP32 (SoftAP) – zmień na swój
DEFAULT_CELL_MM    = 50              # bok komórki siatki [mm]  → 5 cm = 1 kropka
DEFAULT_MAX_RANGE  = 6.0             # maks. zasięg rysowania [m]
DEFAULT_MIN_RANGE  = 0.05            # minimalna sensowna odległość [m] (filtr szumu)
POLL_INTERVAL_S    = 0.08            # odstęp między zapytaniami HTTP
HTTP_TIMEOUT_S     = 1.5             # timeout pojedynczego zapytania
REDRAW_INTERVAL_MS = 150             # odświeżanie wykresu (GUI)


class LidarClient:
    """Wątek odpytujący firmware i akumulujący punkty w siatce zajętości."""

    def __init__(self):
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()

        # Konfiguracja (czytana/zmieniana z wątku GUI – chroniona lockiem).
        self.host = DEFAULT_HOST
        self.cell_mm = DEFAULT_CELL_MM
        self.max_range_mm = DEFAULT_MAX_RANGE * 1000.0
        self.min_range_mm = DEFAULT_MIN_RANGE * 1000.0
        self.angle_offset_deg = 0.0
        self.invert_dir = False

        # Stan mapy (TYLKO bieżący skan – bez akumulacji).
        self.cells = set()           # komórki zajęte w AKTUALNYM skanie (ix, iy)
        self.last_scan = []          # surowe punkty aktualnego skanu [(x_mm, y_mm), ...]
        self.seq = 0                 # ostatni numer sekwencyjny (do statusu)
        self.rpm = 0
        self.connected = False
        self.last_error = ""
        self.n_points = 0            # liczba punktów w bieżącym skanie
        self.pps = 0.0                # punktów/s (tempo napływu danych z lidaru)
        self._last_ingest_t = None    # do liczenia pps (czas poprzedniego _ingest)

    # ── sterowanie wątkiem ──────────────────────────────────────────
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        with self._lock:
            self.connected = False

    def reset_map(self):
        # W trybie bieżącym mapa i tak odświeża się sama; RESET po prostu
        # natychmiast czyści widok do następnego skanu.
        with self._lock:
            self.cells = set()
            self.last_scan = []
            self.n_points = 0

    # ── aktualizacja konfiguracji (z GUI) ───────────────────────────
    def configure(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)

    # ── migawka stanu do rysowania ──────────────────────────────────
    def snapshot(self):
        with self._lock:
            cell = self.cell_mm
            # Środki zajętych komórek BIEŻĄCEGO skanu [m].
            if self.cells:
                arr = np.array(list(self.cells), dtype=np.float64)
                grid_xy = (arr + 0.5) * cell / 1000.0
            else:
                grid_xy = np.empty((0, 2))
            live = (np.array(self.last_scan, dtype=np.float64) / 1000.0
                    if self.last_scan else np.empty((0, 2)))
            return {
                "grid": grid_xy,
                "live": live,
                "rpm": self.rpm,
                "connected": self.connected,
                "n_cells": len(self.cells),
                "n_points": self.n_points,
                "pps": self.pps,
                "error": self.last_error,
            }

    # ── pętla robocza ───────────────────────────────────────────────
    def _run(self):
        while not self._stop.is_set():
            t0 = time.time()
            try:
                with self._lock:
                    host = self.host
                # since=0 => firmware zwraca NAJNOWSZE ~512 punktów (≈1 obrót),
                # czyli zawsze aktualny pełny skan – bez akumulacji historii.
                url = f"http://{host}/api/lidar/scan?since=0"
                with urllib.request.urlopen(url, timeout=HTTP_TIMEOUT_S) as r:
                    data = json.loads(r.read().decode("utf-8"))
                self._ingest(data)
                with self._lock:
                    self.connected = True
                    self.last_error = ""
            except (URLError, OSError, ValueError, json.JSONDecodeError) as e:
                with self._lock:
                    self.connected = False
                    self.last_error = str(e)
                time.sleep(0.4)      # po błędzie nie zalewaj zapytaniami

            dt = time.time() - t0
            if dt < POLL_INTERVAL_S:
                self._stop.wait(POLL_INTERVAL_S - dt)

    # ── przetworzenie odpowiedzi: TYLKO bieżący skan ────────────────
    def _ingest(self, data):
        pts = data.get("pts", [])
        seq = int(data.get("seq", 0))
        rpm = int(data.get("rpm", 0))

        with self._lock:
            cell = self.cell_mm
            off = math.radians(self.angle_offset_deg)
            sign = -1.0 if self.invert_dir else 1.0
            max_r = self.max_range_mm
            min_r = self.min_range_mm

            fresh = []
            cells = set()
            # pts to spłaszczona tablica: [a0, d0, a1, d1, ...]
            for i in range(0, len(pts) - 1, 2):
                ang_h = pts[i]
                dist = pts[i + 1]
                if dist <= 0 or dist < min_r or dist > max_r:
                    continue
                theta = sign * (ang_h / 100.0) * math.pi / 180.0 + off
                x = dist * math.cos(theta)
                y = dist * math.sin(theta)
                fresh.append((x, y))
                cells.add((math.floor(x / cell), math.floor(y / cell)))

            # ZASTĘPUJEMY (nie akumulujemy) – pokazujemy tylko to, co teraz.
            self.last_scan = fresh
            self.cells = cells
            self.n_points = len(fresh)
            self.seq = seq
            self.rpm = rpm

            # Tempo napływu danych [pkt/s] – wygładzone (EMA), żeby liczba
            # w statusie nie skakała przy każdym odczycie.
            now_t = time.time()
            if self._last_ingest_t is not None:
                dt = now_t - self._last_ingest_t
                if dt > 0:
                    inst_pps = len(fresh) / dt
                    self.pps = 0.7 * self.pps + 0.3 * inst_pps
            self._last_ingest_t = now_t

    # ── eksport BIEŻĄCEGO skanu do CSV (środki komórek, metry) ──────
    def export_csv(self, path):
        with self._lock:
            cell = self.cell_mm
            cells = list(self.cells)
        with open(path, "w", encoding="utf-8") as f:
            f.write("x_m,y_m\n")
            for ix, iy in cells:
                f.write(f"{(ix + 0.5) * cell / 1000.0:.3f},"
                        f"{(iy + 0.5) * cell / 1000.0:.3f}\n")
        return len(cells)


class App:
    def __init__(self, root):
        self.root = root
        self.client = LidarClient()
        root.title("Mapa przeszkód – LIDAR")
        root.geometry("1080x760")

        self._build_controls()
        self._build_plot()
        self._schedule_redraw()

    # ── panel sterowania ────────────────────────────────────────────
    def _build_controls(self):
        bar = ttk.Frame(self.root, padding=8)
        bar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(bar, text="Adres robota:").grid(row=0, column=0, sticky="w")
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(bar, textvariable=self.host_var, width=16).grid(row=0, column=1, padx=4)

        self.start_btn = ttk.Button(bar, text="▶ START", command=self.toggle)
        self.start_btn.grid(row=0, column=2, padx=4)
        ttk.Button(bar, text="🗑 RESET mapy", command=self.reset).grid(row=0, column=3, padx=4)
        ttk.Button(bar, text="💾 Eksport CSV", command=self.export).grid(row=0, column=4, padx=4)
        ttk.Button(bar, text="⊡ Dopasuj widok", command=self.fit_view).grid(row=0, column=5, padx=4)

        # Drugi rząd – parametry.
        self.cell_var   = tk.IntVar(value=DEFAULT_CELL_MM)
        self.range_var  = tk.DoubleVar(value=DEFAULT_MAX_RANGE)
        self.offset_var = tk.DoubleVar(value=0.0)
        self.invert_var = tk.BooleanVar(value=False)
        self.live_var   = tk.BooleanVar(value=True)

        ttk.Label(bar, text="Komórka [mm]:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Spinbox(bar, from_=10, to=500, increment=5, width=6,
                    textvariable=self.cell_var, command=self.apply_cfg
                    ).grid(row=1, column=1, sticky="w", pady=(6, 0))

        ttk.Label(bar, text="Zasięg [m]:").grid(row=1, column=2, sticky="e", pady=(6, 0))
        ttk.Spinbox(bar, from_=0.5, to=30, increment=0.5, width=6,
                    textvariable=self.range_var, command=self.apply_cfg
                    ).grid(row=1, column=3, sticky="w", pady=(6, 0))

        ttk.Label(bar, text="Offset kąta [°]:").grid(row=1, column=4, sticky="e", pady=(6, 0))
        ttk.Spinbox(bar, from_=-180, to=180, increment=1, width=6,
                    textvariable=self.offset_var, command=self.apply_cfg
                    ).grid(row=1, column=5, sticky="w", pady=(6, 0))

        ttk.Checkbutton(bar, text="Odwróć kierunek", variable=self.invert_var,
                        command=self.apply_cfg).grid(row=1, column=6, padx=8, pady=(6, 0))
        ttk.Checkbutton(bar, text="Pokaż bieżący skan", variable=self.live_var
                        ).grid(row=1, column=7, padx=4, pady=(6, 0))

        # Pasek statusu.
        self.status = ttk.Label(self.root, text="● Rozłączono", padding=(8, 4))
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

    # ── wykres ──────────────────────────────────────────────────────
    def _build_plot(self):
        self.fig = Figure(figsize=(7, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_facecolor("#fafafa")
        self.ax.grid(True, color="#e0e0e0", linewidth=0.5)
        self.ax.set_xlabel("X [m]  (przód robota →)")
        self.ax.set_ylabel("Y [m]  (← lewo)")
        self.ax.set_title("Bieżące przeszkody (każda kropka = zajęta komórka 5 cm)")

        r = DEFAULT_MAX_RANGE
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)

        # Warstwy: bieżące przeszkody (czarne komórki) + surowy skan (niebieskie).
        self.grid_sc = self.ax.scatter([], [], s=10, c="black", marker="o",
                                       label="przeszkody 5 cm (teraz)")
        self.live_sc = self.ax.scatter([], [], s=6, c="#3a86ff", alpha=0.45,
                                       label="surowy skan")
        # Robot w środku układu + strzałka kierunku.
        self.ax.plot(0, 0, marker="^", color="#e63946", markersize=12, zorder=5)
        self.ax.annotate("", xy=(0.5, 0), xytext=(0, 0),
                         arrowprops=dict(arrowstyle="->", color="#e63946", lw=1.5))
        self.ax.legend(loc="upper right", fontsize=8)

        wrap = ttk.Frame(self.root)
        wrap.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=wrap)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, wrap)  # zoom/pan/zapis PNG

    # ── akcje ───────────────────────────────────────────────────────
    def toggle(self):
        if self.client._thread and self.client._thread.is_alive():
            self.client.stop()
            self.start_btn.config(text="▶ START")
        else:
            self.apply_cfg()
            self.client.configure(host=self.host_var.get().strip())
            self.client.start()
            self.start_btn.config(text="⏸ STOP")

    def reset(self):
        self.client.reset_map()

    def apply_cfg(self):
        try:
            cell = max(5, int(self.cell_var.get()))
            rng = max(0.2, float(self.range_var.get()))
            off = float(self.offset_var.get())
        except (tk.TclError, ValueError):
            return
        self.client.configure(
            cell_mm=cell,
            max_range_mm=rng * 1000.0,
            angle_offset_deg=off,
            invert_dir=bool(self.invert_var.get()),
        )

    def fit_view(self):
        snap = self.client.snapshot()
        pts = snap["grid"]
        if len(pts) == 0:
            r = float(self.range_var.get())
            self.ax.set_xlim(-r, r)
            self.ax.set_ylim(-r, r)
        else:
            m = 0.3
            self.ax.set_xlim(pts[:, 0].min() - m, pts[:, 0].max() + m)
            self.ax.set_ylim(pts[:, 1].min() - m, pts[:, 1].max() + m)
        self.canvas.draw_idle()

    def export(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            title="Zapisz mapę przeszkód")
        if not path:
            return
        n = self.client.export_csv(path)
        messagebox.showinfo("Eksport", f"Zapisano {n} komórek do:\n{path}")

    # ── pętla odświeżania GUI ───────────────────────────────────────
    def _schedule_redraw(self):
        self._redraw()
        self.root.after(REDRAW_INTERVAL_MS, self._schedule_redraw)

    def _redraw(self):
        snap = self.client.snapshot()

        self.grid_sc.set_offsets(snap["grid"] if len(snap["grid"]) else np.empty((0, 2)))
        if self.live_var.get():
            self.live_sc.set_offsets(snap["live"] if len(snap["live"]) else np.empty((0, 2)))
        else:
            self.live_sc.set_offsets(np.empty((0, 2)))

        running = self.client._thread and self.client._thread.is_alive()
        if not running:
            self.status.config(text="● Zatrzymano", foreground="#777")
        elif snap["connected"]:
            self.status.config(
                text=(f"● Połączono   |   przeszkód (kropek): {snap['n_cells']}   |   "
                      f"{snap['rpm']} RPM   |   {snap['pps']:.0f} pkt/s"),
                foreground="#2a9d2a")
        else:
            err = snap["error"] or "brak odpowiedzi"
            self.status.config(text=f"● Łączenie… ({err})", foreground="#c0392b")

        self.canvas.draw_idle()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()