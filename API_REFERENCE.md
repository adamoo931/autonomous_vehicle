# Autonomous Vehicle - HTTP API Reference

## Base URL
```
http://<robot-ip>/api
Default: http://192.168.137.51/api
```

---

## LIDAR & SENSING

### GET /api/lidar/scan
**Full LiDAR rotation with collision/edge status**

**Query Parameters:**
- `since` (optional, default=0): Sequence number to fetch only new points

**Response:**
```json
{
  "seq": 12345,
  "rpm": 800,
  "yaw_rad": 0.523599,
  "edge_detected": false,
  "collision": false,
  "n": 120,
  "pts": [
    100, 500,
    150, 480,
    200, 460,
    ...
  ]
}
```

**Field Descriptions:**
- `seq` (uint32): Sequence number for incremental polling
- `rpm` (uint16): Current LiDAR rotation speed
- `yaw_rad` (float): Robot heading from IMU
- `edge_detected` (bool): CNY70 edge sensor triggered
- `collision` (bool): LiDAR detected obstacle < 200mm in front
- `n` (uint16): Number of points in array
- `pts` (array): [angle_hundredths, distance_mm, angle_hundredths, distance_mm, ...]
  - `angle_hundredths`: Angle × 100 (0-35999 = 0°-359.99°)
  - `distance_mm`: Distance in millimeters (0 = no echo)

**Example Usage (Python):**
```python
import json, urllib.request

url = "http://192.168.137.51/api/lidar/scan?since=0"
with urllib.request.urlopen(url) as resp:
    data = json.loads(resp.read().decode("utf-8"))
    print(f"Points: {data['n']}, Edge: {data['edge_detected']}, Collision: {data['collision']}")
```

---

### GET /api/sensors
**All sensor readings (LiDAR, IMU, pyrometer, odometry, etc.)**

**Response Structure:**
```json
{
  "lidar": {
    "min_distance_mm": 300,
    "angle_hundredths": 18050,
    "speed_rpm": 800,
    "valid": true
  },
  "pyrometer": {
    "object_temp": 23.5,
    "ambient_temp": 20.1,
    "finish_detected": false,
    "initialized": true
  },
  "imu": {
    "accel_x": 0.05, "accel_y": -0.02, "accel_z": 9.81,
    "gyro_x": 0.1, "gyro_y": -0.05, "gyro_z": 0.3,
    "temp": 25.0,
    "initialized": true
  },
  "ina219": {
    "bus_voltage_v": 5.12,
    "shunt_voltage_mv": 12.5,
    "current_ma": 250,
    "power_mw": 1280,
    "address": "0x45",
    "initialized": true
  },
  "sht40": {
    "temperature_c": 22.1,
    "humidity_pct": 45.3,
    "address": "0x44",
    "initialized": true
  },
  "odometry": {
    "pulses_left": 1250,
    "pulses_right": 1248,
    "dist_left_mm": 68750.0,
    "dist_right_mm": 68640.0,
    "dist_total_mm": 137390.0,
    "finish_detected": false
  },
  "line_sensors": {
    "front_left": false,
    "front_right": false,
    "back_left": false,
    "back_right": false
  },
  "motors": {
    "left": 0,
    "right": 0
  },
  "autonomy": {
    "enabled": false,
    "state": "Bezczynny",
    "log_count": 0,
    "remote_collision": false,
    "remote_edge": false
  }
}
```

---

## MOTOR CONTROL

### POST /api/motor
**Set motor speeds**

**Request:**
```json
{
  "left": 40,
  "right": 40
}
```

**Parameters:**
- `left` (int, -100 to 100): Left motor speed percentage
  - Negative = backward, Positive = forward
- `right` (int, -100 to 100): Right motor speed percentage

**Response:**
```json
{
  "ok": true
}
```

**Example (turn left):**
```python
import json, urllib.request

payload = json.dumps({"left": -30, "right": 30}).encode()
req = urllib.request.Request(
    "http://192.168.137.51/api/motor",
    data=payload,
    headers={"Content-Type": "application/json"},
    method="POST"
)
urllib.request.urlopen(req)
```

---

### POST /api/motor/stop
**Stop both motors immediately**

**Request:** (no body)

**Response:**
```json
{
  "ok": true
}
```

---

## AUTONOMOUS NAVIGATION

### POST /api/autonomy/remote_move
**Execute timed motor movement with safety monitoring**

**Request:**
```json
{
  "pwm_l": 40,
  "pwm_r": 40,
  "duration_ms": 380
}
```

**Parameters:**
- `pwm_l` (int, -100 to 100): Left motor PWM
- `pwm_r` (int, -100 to 100): Right motor PWM
- `duration_ms` (uint32): Movement duration in milliseconds

**Response:**
```json
{
  "ok": true,
  "accepted": true,
  "collision": false,
  "edge_detected": false
}
```

**Response Fields:**
- `ok` (bool): Command accepted (will execute or already executing)
- `accepted` (bool): Same as `ok`
- `collision` (bool): **true** if LiDAR front arc < 200mm was detected during move
- `edge_detected` (bool): **true** if CNY70 edge sensor was triggered during move

**Behavioral Notes:**
- If another move is already executing, returns `"accepted": false`
- Movement happens asynchronously; command returns immediately
- Collision/edge status is from during/after the move
- Motors are guaranteed to stop after `duration_ms`
- If collision detected: motors stop immediately, status returned

**Example (forward move, 380ms):**
```python
payload = {
    "pwm_l": 40,
    "pwm_r": 40,
    "duration_ms": 380
}
resp = http_post("/api/autonomy/remote_move", payload)
if resp["collision"]:
    print("⚠️ Obstacle detected during move")
if resp["edge_detected"]:
    print("🔴 Edge/fall detected!")
```

**Example (rotate left, 220ms):**
```python
payload = {
    "pwm_l": -35,
    "pwm_r": 35,
    "duration_ms": 220
}
```

---

### POST /api/autonomy
**Enable/disable autonomous mode**

**Request:**
```json
{
  "enable": true
}
```

**Parameters:**
- `enable` (bool): true = start autonomy, false = stop autonomy
- If no body: toggles current state

**Response:**
```json
{
  "enabled": true,
  "state": "Jazda"
}
```

**States:**
- `"Bezczynny"` - Idle
- `"Jazda"` - Driving forward
- `"Omijanie przeszkody"` - Obstacle avoidance
- `"Cofanie"` - Backing up
- `"Obrot"` - Rotating
- `"Remote Exec"` - Remote move executing
- `"Remote Wait"` - Waiting after remote move
- `"META osiagnieta"` - Goal reached (thermal target found)
- `"Awaria (utkniecie)"` - Fault state (stuck too long)

---

### GET /api/autonomy/log.csv
**Download autonomy mission log**

**Headers:**
```
Content-Type: text/csv; charset=utf-8
Content-Disposition: attachment; filename="przejazd_log.csv"
```

**CSV Format:**
```
czas_ms,stan,silnik_L_proc,silnik_R_proc,lidar_przod_mm,lidar_diag_L_mm,lidar_diag_R_mm,lidar_bok_L_mm,lidar_bok_R_mm,najwiecej_miejsca_st,temp_obiekt_C,temp_otoczenie_C,delta_C
0,Bezczynny,0,0,500,800,800,1200,1200,0,22.5,20.0,2.5
50,Jazda,35,35,450,750,850,1100,1200,0,22.4,20.0,2.4
100,Jazda,37,33,400,700,900,1050,1250,5,22.3,20.0,2.3
...
```

**Fields:**
- `czas_ms`: Time since mission start (milliseconds)
- `stan`: State name
- `silnik_L_proc`, `silnik_R_proc`: Motor speeds (-100 to 100%)
- `lidar_*_mm`: LiDAR distances in millimeters
- `najwiecej_miejsca_st`: Most open direction (degrees)
- `temp_*_C`: Temperature in Celsius
- `delta_C`: Thermal delta (object - ambient)

**Usage:**
```python
import requests

# Download log
response = requests.get("http://192.168.137.51/api/autonomy/log.csv")
with open("mission_log.csv", "wb") as f:
    f.write(response.content)

# Analyze in Pandas
import pandas as pd
df = pd.read_csv("mission_log.csv")
print(f"Duration: {df['czas_ms'].max() / 1000:.1f} seconds")
print(f"Max distance (front): {df['lidar_przod_mm'].min()} mm")
```

---

## LED CONTROL

### POST /api/led
**Control RGB LEDs**

**Request:**
```json
{
  "red": 1,
  "yellow": 0,
  "green": 1
}
```

**Parameters:**
- `red`, `yellow`, `green` (int, 0 or 1): LED state (on/off)

---

## BUZZER CONTROL

### POST /api/buzzer
**Play tone from passive buzzer**

**Request (default beep):**
```json
{}
```

**Request (custom frequency/duration):**
```json
{
  "freq": 2000,
  "duration_ms": 200
}
```

**Parameters:**
- `freq` (optional, int): Frequency in Hz (default 2000)
- `duration_ms` (optional, int): Duration in milliseconds (default 150)

---

## ODOMETRY

### POST /api/odometry/reset
**Reset odometry counters**

**Request:** (no body)

**Response:**
```json
{
  "ok": true
}
```

---

## LOGGING

### GET /api/logs
**Retrieve recent console logs**

**Response:**
```text
I [MOTOR] Motor driver initialized
E [LIDAR] UART read error (check baud rate)
W [AUTO] Stuck 1600ms no movement - escape attempt
I [AUTO] [Jazda] silniki L=35% R=35% | lidar mm: przod=500 ...
```

**Format:** Plain text, UART log lines with level prefix:
- `I` = Info
- `W` = Warning
- `E` = Error
- `D` = Debug

### POST /api/logs/clear
**Clear log buffer**

**Request:** (no body)

**Response:**
```json
{
  "ok": true
}
```

---

## ERROR RESPONSES

### Standard Error Response:
```json
{
  "error": "description"
}
```

### HTTP Status Codes:
- `200 OK` - Request successful
- `400 Bad Request` - Invalid JSON or parameters
- `404 Not Found` - Endpoint doesn't exist
- `500 Internal Server Error` - ESP32 error or out of memory

---

## RESPONSE TIME EXPECTATIONS

| Endpoint | Typical Response Time |
|----------|----------------------|
| `/api/sensors` | 10-50ms |
| `/api/lidar/scan` | 20-100ms |
| `/api/motor` | 5-20ms |
| `/api/autonomy/remote_move` | 10-30ms |
| `/api/autonomy/log.csv` | 50-500ms (depends on log size) |

---

## RATE LIMITING

- No built-in rate limiting on ESP32
- Recommended polling intervals:
  - **Sensors**: 1 Hz (100ms minimum)
  - **LiDAR**: 5-10 Hz (100-200ms minimum)
  - **Remote moves**: Sequential only (wait for previous move to complete)

---

## PYTHON CLIENT EXAMPLE

```python
import json
import urllib.request
import time

class RobotAPI:
    def __init__(self, host="192.168.137.51"):
        self.host = host
        self.timeout = 0.8
    
    def _get(self, path):
        url = f"http://{self.host}/api{path}"
        try:
            with urllib.request.urlopen(url, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            print(f"GET error: {e}")
            return None
    
    def _post(self, path, data):
        url = f"http://{self.host}/api{path}"
        try:
            payload = json.dumps(data).encode("utf-8")
            req = urllib.request.Request(
                url, data=payload,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            print(f"POST error: {e}")
            return None
    
    def get_sensors(self):
        return self._get("/sensors")
    
    def move_forward(self, duration_ms=380, speed=40):
        """Drive forward with safety monitoring"""
        resp = self._post("/autonomy/remote_move", {
            "pwm_l": speed,
            "pwm_r": speed,
            "duration_ms": duration_ms
        })
        if resp:
            if resp.get("collision"):
                print("⚠️ Collision detected!")
            if resp.get("edge_detected"):
                print("🔴 Edge detected!")
            return resp.get("ok", False)
        return False
    
    def rotate(self, direction=1, duration_ms=220, speed=35):
        """Rotate in place (direction: 1=left, -1=right)"""
        return self._post("/autonomy/remote_move", {
            "pwm_l": -direction * speed,
            "pwm_r": direction * speed,
            "duration_ms": duration_ms
        }).get("ok", False)
    
    def stop(self):
        return self._post("/motor/stop", {}).get("ok", False)
    
    def get_lidar_scan(self):
        return self._get("/lidar/scan?since=0")

# Usage
robot = RobotAPI("192.168.137.51")

# Get sensor data
sensors = robot.get_sensors()
print(f"LiDAR min distance: {sensors['lidar']['min_distance_mm']}mm")

# Move with safety
if robot.move_forward(duration_ms=380, speed=40):
    print("Move executed")
    time.sleep(0.5)  # Wait for movement
else:
    print("Move failed")

# Rotate
robot.rotate(direction=1, duration_ms=220)  # Rotate left
time.sleep(0.3)

# Emergency stop
robot.stop()
```

---

## DEBUGGING TIPS

### Check Connectivity:
```bash
curl http://192.168.137.51/api/sensors
```

### Log streaming:
```bash
watch -n 0.1 curl http://192.168.137.51/api/logs
```

### Monitor LiDAR:
```python
import json, urllib.request
for i in range(100):
    data = json.loads(urllib.request.urlopen("http://192.168.137.51/api/lidar/scan?since=0").read())
    print(f"Points: {data['n']}, Collision: {data['collision']}, Edge: {data['edge_detected']}")
    time.sleep(0.1)
```

