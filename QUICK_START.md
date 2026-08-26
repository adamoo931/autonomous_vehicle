# Autonomous Vehicle - Quick Start & Deployment Guide

## Prerequisites

### Hardware Requirements:
- ESP32 microcontroller (with LiDAR LD06/LD14P, IMU ICM-20948, pirometer MLX90614)
- LiDAR sensor (LD06 or LD19) at 230400 baud, UART2
- CNY70 reflective sensors on GPIO 14, 39, 15, 23
- TB6612FNG motor driver
- DC motors with Hall effect odometry sensors

### Software Requirements:
- ESP-IDF v4.4+ (ESP32 development environment)
- Python 3.8+
- pip packages:
  - numpy
  - scipy
  - matplotlib
  - tkinter (usually bundled with Python)

---

## FIRMWARE BUILD & DEPLOYMENT

### 1. Set Up ESP-IDF Environment
```bash
# Install ESP-IDF (if not already done)
mkdir ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git -b v4.4-stable

# Setup environment (Windows - use ESP-IDF PowerShell or CMD)
cd esp-idf
install.bat  # Windows
# OR
. ./install.sh && . ./export.sh  # Linux/Mac
```

### 2. Prepare Project
```bash
cd c:\autonomiczny_pojazd\autonomous_vehicle

# Configure for your board
idf.py set-target esp32

# Configure build settings (WiFi SSID, baud rates, etc.)
idf.py menuconfig
# Navigate to: Project Configuration (Config) → WiFi SSID/Password
```

### 3. Build Firmware
```bash
# Full rebuild
idf.py build

# Clean and rebuild (if needed)
idf.py fullclean
idf.py build

# Monitor build output (verbose)
idf.py build -v
```

### 4. Flash to ESP32
```bash
# Flash and monitor serial output
idf.py -p COM3 flash monitor
# (Replace COM3 with your ESP32's serial port)

# Flash only (without monitor)
idf.py -p COM3 flash

# Monitor existing serial (if already flashed)
idf.py -p COM3 monitor
```

### 5. Verify Firmware
- Serial monitor should show:
  ```
  [LIDAR] LiDAR initialized...
  [MOTOR] Motor driver initialized
  [AUTO] Moduł autonomii gotowy (wylaczony)...
  [HTTP] HTTP server running on port 80
  ```
- Connect browser to `http://192.168.137.51` (or robot's IP)
- Dashboard should load with sensor readings

---

## PYTHON APPLICATION SETUP

### 1. Install Dependencies
```bash
cd main

# Create virtual environment (optional but recommended)
python -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows

# Install packages
pip install numpy scipy matplotlib
```

### 2. Configure Robot IP
Edit `lidar_map.py` line ~23:
```python
DEFAULT_HOST = "192.168.137.51"  # Change to your robot's IP
```

### 3. Run Application
```bash
python lidar_map.py
```

### Expected GUI:
- Window titled "Autonomous Vehicle – Enhanced SLAM Navigation"
- Large map visualization (white area in center)
- Control buttons: START SLAM, STOP, CLEAR MAP, START NAVIGATION
- Status bar at bottom with position/angle/RMSE

---

## OPERATION GUIDE

### Basic Workflow:

#### 1. Start SLAM
```
Button: "▶ START SLAM"
```
- Robot begins 360° LiDAR scans
- Map builds in real-time as robot explores
- Status shows: "BOOTSTRAPPING..." then "SLAM OK"
- GUI displays:
  - Gray areas: explored free space
  - Black areas: obstacles
  - Red dot: robot position
  - Red arrow: robot heading

#### 2. Set Goal
```
Mouse: Click on map where you want robot to go
```
- Blue X marker appears at click position
- Buttons enable: "🚀 START NAVIGATION"
- Path displays as yellow line from robot to goal

#### 3. Start Navigation
```
Button: "🚀 START NAVIGATION"
```
- Robot executes Stop-and-Go navigation:
  1. Full 360° scan
  2. Proximity Guard check (abort if < 20cm ahead)
  3. Path plan to goal
  4. Execute one maneuver (rotate or forward)
  5. Repeat until goal reached
- Status updates: "NAVIGATING: Rotating θ°" or "NAVIGATING: Moving forward"

#### 4. Monitor Safety
Watch for:
- **"🔴 EDGE"** - Edge sensor triggered
- **"💥 COLLISION"** - LiDAR detected obstacle
- **"PROXIMITY GUARD: dist=X"** - Obstacle too close, executing correction
- **RMSE** value (should stay < 0.085m for good localization)
- **Inlier ratio** (should stay > 0.35 for valid poses)

---

## TROUBLESHOOTING

### Firmware Issues:

**"LiDAR not responding"**
- Check UART2 pins (GPIO 1 RX, GPIO 3 TX)
- Verify baud rate: 230400
- Physical connection: GND, 5V, TX, RX

**"Motor driver not responding"**
- Check GPIO pins (PWM on 19/0, direction on 5/18/4/16)
- Verify TB6612FNG enable pin is HIGH
- Test with manual move command in dashboard

**"ESP32 won't connect to WiFi"**
- Verify SSID/password in menuconfig
- Check WiFi network is 2.4GHz (not 5GHz)
- Reset ESP32, wait 5 seconds for connection

### Python Application Issues:

**"ConnectionError: Cannot connect to robot"**
- Verify robot IP: `DEFAULT_HOST` in lidar_map.py
- Ping robot: `ping 192.168.137.51` (or your IP)
- Check robot's WiFi dashboard loads in browser

**"SLAM OK but not moving"**
- Click on map to set goal
- Button "🚀 START NAVIGATION" should enable
- If still disabled: check "RMSE" value (might be > 0.085, quality gate failing)

**"Proximity Guard triggering constantly"**
- May indicate poor LiDAR data or map noise
- Try: `CLEAR MAP`, re-run SLAM with obstacle-free area
- Check sensor calibration (LiDAR offset, orientation)

**"Memory Error in Python"**
- Check available RAM (map = ~50MB for full grid)
- If on low-memory system: reduce `GRID_WIDTH`, `GRID_HEIGHT` to 300×300

---

## CONFIGURATION TUNING

### Firmware (in `main/autonomy.c`):

**Drive Speed:**
```c
#define SP_CRUISE        35      // Jazz speed (0-100%)
#define KICK_POWER       90      // Startup impulse
#define KICK_MS         220      // Impulse duration
```

**Collision Detection:**
```c
// Change from 200mm to custom value (in ST_REMOTE_EXEC):
if (front_collision_arc < 250)  // 250mm instead of 200mm
```

**Sensor Calibration:**
```c
#define LID_FRONT_DEG    0       // LiDAR offset angle
#define LID_MIRROR       0       // Set to 1 if turning opposite direction
```

### Python (in `lidar_map.py`):

**Safety Distance:**
```python
PROXIMITY_GUARD_M = 0.20         # Change from 20cm to custom
PROXIMITY_GUARD_ANGLE = math.radians(35)  # Front arc size
```

**Localization Quality:**
```python
ICP_MAX_RMSE = 0.085             # Stricter = more conservative
ICP_MIN_INLIER_RATIO = 0.35      # Higher = more robust
```

**Exploration Penalty:**
```python
# In OptimisticPathPlanner.plan():
unknown_penalty = 1.4 if cell_val == -1 else 1.0
# Higher = avoid unknown areas; Lower = explore more
```

**Path Planning:**
```python
target_idx = min(6, len(self.path_cells) - 1)
# 6 = lookahead distance (cells); larger = smoother, slower response
```

---

## DATA LOGGING & ANALYSIS

### Firmware Logs:
1. **Web Dashboard**: `http://robot-ip/` → Monitor panel at bottom
2. **Serial Monitor**: `idf.py monitor` → Real-time ESP32 logs
3. **Autonomy Log**: Dashboard → "Pobierz log przejazdu (CSV)" → Excel/Python analysis

### Python Session:
```python
# Modify AutonomousWorker to save pose history
poses_history = []  # Add to __init__
poses_history.append(self.pose.copy())  # Add in run() loop

# Save after session:
import pickle
pickle.dump(poses_history, open("trajectory.pkl", "wb"))

# Analyze:
import numpy as np
poses = np.array(poses_history)
print(f"Total distance: {np.sum(np.linalg.norm(np.diff(poses[:,:2], axis=0), axis=1)):.2f}m")
```

---

## PERFORMANCE OPTIMIZATION

### If SLAM is slow:

1. **Reduce map resolution** (bigger cells):
   ```python
   CELL_SIZE_M = 0.10  # 10cm instead of 5cm
   GRID_WIDTH = 200    # 200×200 cells
   ```

2. **Reduce sample size** (fewer ICP points):
   ```python
   pts = self.map_mgr.get_occupied_points(max_points=1000)  # 1000 instead of 3000
   ```

3. **Skip localization** (faster but less safe):
   ```python
   if not self.bootstrapped or len(map_pts) < 10:  # Lower threshold
   ```

### If navigation is too slow:

1. **Increase lookahead** (fewer waypoints):
   ```python
   target_idx = min(10, len(self.path_cells) - 1)  # 10 instead of 6
   ```

2. **Increase drive time** (longer steps):
   ```python
   payload = {"pwm_l": 40, "pwm_r": 40, "duration_ms": 500}  # 500 instead of 380
   ```

---

## SAFETY BEST PRACTICES

1. **Always use Proximity Guard** (20cm = robot body size)
   - Don't reduce below robot width
   - Increase in narrow spaces (< 40cm corridors)

2. **Verify ICP quality** before trusting pose
   - RMSE should stay < 8.5cm
   - Inlier ratio should stay > 35%
   - Watch status bar in GUI

3. **Start in open space**
   - Clear area > 1m × 1m for initialization
   - Avoid repeating textures (white walls confuse ICP)
   - Ensure LiDAR is 10-20cm above ground

4. **Test manually first**
   - Use dashboard to move robot around manually
   - Verify motors, sensors, WiFi working
   - Then enable autonomous navigation

5. **Monitor first run**
   - Stand near robot during first autonomy test
   - Have manual control ready (stop button on dashboard)
   - Watch for stuck states (SLAM rejected, path planning failures)

---

## EXAMPLE SESSION

```bash
# Terminal 1: Monitor ESP32
cd c:\autonomiczny_pojazd\autonomous_vehicle
idf.py monitor

# Terminal 2: Run Python app
cd c:\autonomiczny_pojazd\autonomous_vehicle\main
python lidar_map.py

# In GUI:
1. Click "START SLAM" button
   → See robot position as red dot, gray map building
   → Wait 5-10 seconds for good map coverage
   → Status: "SLAM OK"

2. Click on unexplored area (dark blue) as goal
   → Blue X appears
   → "START NAVIGATION" button enables

3. Click "START NAVIGATION"
   → Robot status: "NAVIGATING: Rotating 45°"
   → Robot starts Stop-and-Go
   → Watch for:
      - Proximity Guard corrections (small turns)
      - Path being executed (yellow line)
      - RMSE < 0.085m (good localization)

4. Robot reaches goal
   → Status: "GOAL REACHED!"
   → Stops and awaits new goal
```

---

## SUPPORT & DEBUGGING

### Useful Serial Commands (if needed):
```
idf.py monitor -p COM3 --baud 115200
```

### Key Files to Check:
- Firmware errors: `build/compile_commands.json`
- Config: `sdkconfig` (human-readable settings)
- Python output: Run in terminal to see full traceback

### Online Resources:
- ESP-IDF: https://docs.espressif.com/projects/esp-idf/
- LiDAR LD06: Check manufacturer datasheet (230400 baud is standard)
- Python SLAM: https://github.com/introlab/rtabmap for reference

---

## FINAL CHECKLIST

- [ ] Firmware compiles without errors
- [ ] ESP32 flashes and boots successfully
- [ ] WiFi connects and dashboard loads
- [ ] All sensors visible on dashboard (LiDAR, IMU, motors, etc.)
- [ ] Manual motor control works (Dashboard D-Pad)
- [ ] Python environment has numpy, scipy, matplotlib
- [ ] Robot IP matches DEFAULT_HOST in lidar_map.py
- [ ] First SLAM run successful (map builds, RMSE < 0.085m)
- [ ] Navigation works (robot reaches clicked goal)
- [ ] Safety features tested (Proximity Guard, edge detection)

**Status:** Ready for Deployment ✅

