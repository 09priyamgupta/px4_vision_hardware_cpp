# RAPTOR: Real-Time Adaptive Path Planning for Collaborative UAV-Ground Vehicle Rendezvous

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-1A293E.svg)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-orange.svg)](https://gazebosim.org/home)
[![Python](https://img.shields.io/badge/Python-3.10-green.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Project Overview

RAPTOR is a vision-based autonomous landing system for precise UAV-to-ground-vehicle rendezvous using AprilTag fiducial markers. The system combines GPS/INS-based search phases with a vision-guided MPC terminal landing controller, achieving centimeter-level accuracy on both static and moving platforms.

**Hardware-Validated Performance:**
RAPTOR achieves **5.82 cm horizontal landing accuracy on a stationary AprilTag landing pad** using vision-based MPC control on a Raspberry Pi 5 + Pixhawk 6C embedded platform.

## Validation Phases
 
### ✅ Phase I: Simulation (COMPLETE)
- **Environment:** Gazebo with moving UGV (0.50 m/s target velocity)
- **Mean Tracking Error:** 4.3 cm
- **Touchdown Accuracy:** 8.6 cm
- **Purpose:** Proof-of-concept for MPC controller design
- **Status:** Baseline for hardware comparison
### ✅ Phase II: Hardware - Static Platform (COMPLETE, DEFENDED IN THESIS)
- **Environment:** Real X500 + Pixhawk 6C + Raspberry Pi 5
- **Target:** Stationary R1 rover with AprilTag landing pad
- **Mean Tracking Error:** **5.39 cm**
- **Final Touchdown Accuracy:** **5.82 cm**
- **Duration:** 70-second mission (takeoff to touchdown)
- **Key Achievement:** **Hardware system is MORE ACCURATE than simulation** (4.3→5.39 cm acceptable gap for sim-to-real)
- **CPU Load:** 12.6% (Raspberry Pi 5)
- **Status:** ✅ Fully validated, flight-tested
### 🔄 Phase III: Hardware - Moving Platform (FUTURE WORK, NOT IN THESIS)
- **Environment:** Moving R1 rover at configurable velocity
- **Target Velocity:** 0.1–2.0 m/s (planned, not tested)
- **Expected Performance:** <8 cm accuracy (estimated)
- **Status:** 🔄 In development, not part of current defense
- **Roadmap:** 6–12 month extension after thesis completion
---

### System Composition

| Component | Role | Technology |
|-----------|------|-----------|
| **Perception** | AprilTag multi-scale bundle detection | Apriltag ROS 2 + OpenCV (C++) |
| **State Estimation** | EKF-based tag tracking | Kalman Filter + velocity estimation |
| **Control** | Landing trajectory optimization | Model Predictive Control (MPC) via CVXPY |
| **Flight Control** | PX4 autopilot interface | px4_ros2 ModeExecutorBase (C++17) |
| **Logging** | Flight data capture & analysis | CSV @ 30 Hz + ROS 2 bag replay |

---

## Hardware Architecture

### Airframe
- **Platform:** DJI Matrice 300 RTK → **X500 (Pixhawk 6C)**
- **Computer:** Raspberry Pi 5 (8GB) + IP67 housing
- **Camera:** Pi Camera Module 3 (12MP, ~77° HFOV)
- **Communication:** Ethernet UDP (MicroXRCE-DDS) over USB-C dock

### Custom Landing Pad
- **Tag Configuration:**
  - **Tag 19** (50mm): Outer precision marker
  - **Tags 11, 23** (150mm, 400mm): Multi-scale bundle for long-range detection

### Positioning
- **OptiTrack MoCap:** Ground truth for validation (not required in field deployment)
- **Pixhawk EKF2:** NED frame position/velocity estimation
- **Camera Frame:** Z-forward optical convention (rotated to NED via static TF)

---

## Software Stack

### Dependencies
```
ROS 2 Distribution: Humble or Jazzy
Python: 3.10+
C++: C++17 (px4_ros2 requirement)

Key Packages:
- px4_ros2 (flight control abstraction)
- apriltag_ros (tag detection)
- image_proc (camera rectification)
- geometry2 (TF2 frame transformations)
- sensor_msgs, geometry_msgs, px4_msgs
- cvxpy, numpy, scipy (MPC solver)
- rclcpp, rclpy (ROS 2 C++/Python bindings)
```

### Build System
```
colcon (ROS 2 build tool)
CMake 3.22+
Python setuptools
```

---

## Installation & Setup

### 1. **Prerequisites on Raspberry Pi 5**

```bash
# SSH into the drone computer
ssh pi@192.168.1.241
# Password: ifros2023

# Enter the pre-configured Docker container
docker exec -it px4_vision bash

# Verify ROS 2 environment
source /opt/ros/humble/setup.bash
echo $ROS_DOMAIN_ID  # Should be set (typically 42 for px4)
```

### 2. **Clone & Build the Workspace**

```bash
# Inside the Docker container
cd ~/px4_ros2_ws

# Fetch the RAPTOR package (if not already present)
git clone https://github.com/09priyamgupta/px4_vision_hardware_cpp.git \
  src/px4_vision_hardware_cpp

# Build only this package
colcon build --packages-select px4_vision_hardware_cpp

# Source the workspace
source install/setup.bash
```

### 3. **Configuration Files**

Three critical YAML files must be present in `config/`:

#### `config/frames.yaml`
Defines the TF tree structure and sensor frame names:
```yaml
/**:
  ros__parameters:
    frames:
      world: "map"                          # Global reference frame
      drone: "base_link"                    # Drone body frame
      camera: "x500_mono_cam_0/camera_link/imager_optical"
      rover: "r1_rover"
      landing_pad: "landing_pad"            # Tag bundle origin
    vision_timeout: 0.5                     # Max age for vision data (seconds)
    tag_switch_altitude: 0.60               # Altitude to switch from 400mm→50mm tag
```

#### `config/mission_params.yaml`
Mission-specific parameters:
```yaml
/**:
  ros__parameters:
    battery_failsafe_pct: 15.0              # RTB trigger
    target_altitude: 4.0                    # Search phase altitude
    mpc:
      horizon: 20                           # MPC prediction steps
      control_rate: 20.0                    # Hz (20 Hz = 50 ms control loop)
```

#### `config/apriltag.yaml`
AprilTag detector configuration:
```yaml
/**:
  ros__parameters:
    bundle_ids: [11, 23, 19]                # Valid tag IDs in the bundle
```

---

## Running the System

### **Critical: 6-Terminal Deployment Sequence**

The system requires **6 concurrent ROS 2 processes**. Start them in this order with ~2 second intervals:

---

### **Terminal 1: Camera Capture Node**

Streams raw image frames from Pi Camera Module 3 with fixed exposure control.

```bash
ssh pi@192.168.1.241
# Password: ifros2023

docker exec -it px4_vision bash

ros2 run camera_ros camera_node --ros-args \
    -p width:=640 \
    -p height:=480 \
    -p format:="BGR888" \
    -p camera:="/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a" \
    -p frame_id:="x500_mono_cam_0/camera_link/imager" \
    -p focus:=0.0 \
    -p AeEnable:=false \
    -p ExposureTime:=1500 \
    -p AnalogueGain:=4.0
```

**Expected Output:**
```
[camera_node-1] [INFO] Camera initialized: 640x480 @ 30 Hz
[camera_node-1] Publishing to /camera/image_raw and /camera/camera_info
```

**Tuning Notes:**
- `ExposureTime`: 1000–2000 μs (outdoor: higher; indoor: lower)
- `AnalogueGain`: 2–6 (3–4 typical for bright conditions)
- Adjust for AprilTag border sharpness (detector.refine=True amplifies gain)

---

### **Terminal 2: MicroXRCE-DDS Bridge**

Connects Pixhawk 6C (Ethernet UDP) to ROS 2 DDS network. **Critical for PX4 communication.**

```bash
docker exec -it px4_vision bash

sudo MicroXRCEAgent udp4 -p 8888
```

**Expected Output:**
```
[INFO] MicroXRCE Agent version: 2.4.1
[INFO] Listening on UDP port 8888
[INFO] Client connected: 192.168.1.240:xxxxx (Pixhawk 6C)
```

**Troubleshooting:**
- If "Port already in use": `sudo lsof -i :8888` and kill the process
- If client not connecting: Check Pixhawk USB-C cable and IP routing (`ip route`)

---

### **Terminal 3: MoCap Bridge (Optional for Validation)**

Streams OptiTrack ground truth to ROS 2 TF tree. **Skip in field deployment; keep for thesis validation.**

```bash
docker exec -it px4_vision bash

ros2 launch mocap_px4_bridge run.launch.py
```

**Expected Output:**
```
[mocap_px4_bridge-1] [INFO] NatNet protocol initialized
[mocap_px4_bridge-1] [INFO] Rigid bodies: x500_2_priyam, landing_pad, r1_rover
```

**To disable in production:** Comment out in `landing.launch.py` or set `DISABLE_MOCAP=true`.

---

### **Terminal 4: QGroundControl (Optional Desktop)**

Ground station for mission planning, RC control override, and failsafe monitoring.

```bash
# On your desktop (not in Docker)
qgroundcontrol

# Configure:
# - Comm Link → UDP → 192.168.1.241:14550 (adjust IP to your Pixhawk router)
# - Check "Armed", battery %, GPS fix before takeoff
```

---

### **Terminal 5: Launch Main Landing Pipeline**

Activates the complete vision-based landing system: AprilTag detection, landing director, MPC solver, and PX4 mode executor.

```bash
docker exec -it px4_vision bash

ros2 launch px4_vision_hardware_cpp landing.launch.py
```

**Expected Output:**
```
[landing_director-1] [INFO] Landing Director (C++) Started using x500_mono_cam_0/camera_link/imager_optical
[landing_director-1] [INFO] Waiting for tags...

[apriltag_node-3] [INFO] AprilTag detector threads=4, decimate=1.0, refine=True
[apriltag_node-3] [INFO] Detected 1 tags in 18.3ms

[mpc_solver_node-4] [INFO] MPC Solver Bridge Active. Waiting for C++ Mode...

[vision_landing_control_node-5] [INFO] [EXECUTOR] Initialized. Battery Failsafe set to: 15.0%
```

**What this launch does:**
1. ✅ Publishes static TF (drone → camera orientation)
2. ✅ Launches AprilTag detection pipeline (image_proc + apriltag_ros)
3. ✅ Starts landing director (bundle vs. Tag 19 switching)
4. ✅ Activates relative pose node (camera frame → drone frame)
5. ✅ Starts MPC solver (Python CVXPY bridge)
6. ✅ Registers vision landing mode with PX4

**Do NOT start Terminal 6 until you see all 6 INFO messages above.**

---

### **Terminal 6: Data Logging**

Records flight data (30 Hz) to timestamped CSV for post-flight analysis.

```bash
docker exec -it px4_vision bash

./record_flight_data.sh
```

**Script Contents** (create if missing):
```bash
#!/bin/bash

# record_flight_data.sh
ros2 run px4_vision_hardware_cpp landing_data_logger &
LOGGER_PID=$!

# Keep script alive
wait $LOGGER_PID
```

**Output Location:**
```
~/px4_ros2_ws/hardware_analysis/thesis_landing_log_<UNIX_TIMESTAMP>.csv
```

**CSV Columns** (80+ fields):
- `mocap_drone_n`, `mocap_drone_e`, `mocap_drone_d` — Ground truth position (NED)
- `px4_drone_n`, `px4_drone_e`, `px4_drone_d` — EKF estimated position
- `px4_vel_n`, `px4_vel_e`, `px4_vel_d`, `px4_yaw` — Velocity & heading
- `raw_cam_tag_x`, `raw_cam_tag_y`, `raw_cam_tag_z` — Raw tag pose in camera frame
- `kf_target_n`, `kf_target_e` — Kalman-filtered tag position
- `mpc_error_n`, `mpc_error_e`, `total_horiz_error` — Control performance
- `mpc_cmd_vn`, `mpc_cmd_ve` — MPC-computed velocity commands
- `tag_visible`, `using_small_tag`, `landing_funnel_limit` — State machine
- `px4_nav_state`, `custom_mission_state` — Flight mode

---

## System Architecture

### **ROS 2 Node Graph**

```
┌─────────────────────────────────────────────────────────────────┐
│ PERCEPTION LAYER (Real-time image → tag detections)             │
├─────────────────────────────────────────────────────────────────┤
│ camera_node (Pi Camera 3)                                       │
│     ↓ /camera/image_raw, /camera/camera_info                   │
│ image_proc::RectifyNode (distortion correction)                 │
│     ↓ /camera/image_rect                                        │
│ apriltag_ros::AprilTagNode (tag detection @ ~20 Hz)            │
│     ↓ /detections (apriltag_msgs::AprilTagDetectionArray)       │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│ STATE ESTIMATION LAYER (camera → world frame)                   │
├─────────────────────────────────────────────────────────────────┤
│ landing_director (C++) — Tag switching logic                     │
│     input: /detections, drone altitude (EKF)                    │
│     output: /landing_target_pose (selected tag in camera frame) │
│                                                                 │
│ apriltag_relative_pose (C++) — Transform composition            │
│     input: /landing_target_pose                                 │
│     output: /apriltag/relative_pose (drone frame)               │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│ CONTROL LAYER (state → velocity commands)                        │
├─────────────────────────────────────────────────────────────────┤
│ mpc_solver_node (Python CVXPY) — Trajectory optimization         │
│     input: current position, tag velocity, altitude              │
│     output: /mpc/command (vn, ve velocity setpoints)            │
│                                                                 │
│ vision_landing_mode (C++) — Executes MPC commands on PX4        │
│     input: /mpc/command                                         │
│     output: TrajectorySetpoint to Pixhawk 6C via px4_ros2       │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│ MISSION EXECUTOR (lifecycle management)                          │
├─────────────────────────────────────────────────────────────────┤
│ MissionExecutor (C++) — State machine                            │
│     states: VisionLanding → Land → WaitUntilDisarmed            │
│     interfaces: px4_ros2::ModeExecutorBase, PX4 flight modes    │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│ DATA LOGGING                                                    │
├─────────────────────────────────────────────────────────────────┤
│ landing_data_logger (C++) — 30 Hz CSV logging                   │
│     subscribes: all state/control/perception topics             │
│     output: hardware_analysis/thesis_landing_log_*.csv          │
└─────────────────────────────────────────────────────────────────┘
```

### **Key Topic Mappings**

| Topic | Type | Publisher | Subscriber | Rate | Purpose |
|-------|------|-----------|-----------|------|---------|
| `/camera/image_raw` | sensor_msgs::Image | camera_node | image_proc | 30 Hz | Raw video stream |
| `/detections` | apriltag_msgs::AprilTagDetectionArray | apriltag_ros | landing_director | 20 Hz | Raw tag detections |
| `/landing_target_pose` | geometry_msgs::PoseStamped | landing_director | apriltag_relative_pose | 15 Hz | Selected tag (camera frame) |
| `/apriltag/relative_pose` | geometry_msgs::PoseStamped | apriltag_relative_pose | mpc_solver_node | 20 Hz | Tag position (drone frame) |
| `/mpc/state` | std_msgs::Float32MultiArray | vision_landing_mode | mpc_solver_node | 20 Hz | Current position, velocity, altitude |
| `/mpc/command` | std_msgs::Float32MultiArray | mpc_solver_node | vision_landing_mode | 20 Hz | Velocity commands (vn, ve) |
| `/landing/tag_visible_flag` | std_msgs::Bool | apriltag_relative_pose | search_landing_pad_mode | 20 Hz | Tag in FOV indicator |
| `/fmu/out/vehicle_odometry` | px4_msgs::VehicleOdometry | Pixhawk 6C | all nodes | 50 Hz | EKF position, velocity, orientation |

---

## Configuration & Tuning

### **AprilTag Detector Tuning** (`config/apriltag.yaml`)

```yaml
detector:
  threads: 4              # CPU cores for detection (Raspberry Pi 5 has 8)
  decimate: 2.0 OR 1.0    # Downsampling factor
  blur: 0.0               # Gaussian blur (0 = disabled)
  refine: True            # Apply subpixel refinement
  sharpening: 0.5         # Edge sharpening (0–1)

pose_estimation_method: "pnp"  # Perspective-n-Point for 6-DOF

max_hamming: 0            # Reject detections with >0 bit errors (strict)
```

**Dynamic Decimation Logic** (`landing_director.cpp`, line ~120):
- **Above 0.60 m altitude:** `decimate=2.0` (faster, ~20 ms per frame)
- **Below 0.60 m altitude:** `decimate=1.0` (slower, ~35 ms per frame, but pixel-sharp for 50mm tag)

**Hardware Measurement:**
- Decimation switch triggers automatically when drone descends below `tag_switch_altitude` parameter
- **Real performance**: 6.60 cm mean tracking error achieved with adaptive decimation

---

### **MPC Controller Tuning** (`mpc_tracker.py`)

```python
Q = np.eye(2) * 8.0      # Position error weight
                          # Higher → more aggressive centering
                          # Lower → smoother, less reactive

R = np.eye(2) * 14.0     # Velocity effort weight
                          # Higher → constrains max speed
                          # Tuned to match tag chase (1 m/s max)

R_diff = np.eye(2) * 10.0  # Jerk penalty (velocity change)
                          # Higher → smoother deceleration near landing

fov_bound: Dynamic       # Max position error = altitude * tan(FOV/2) * 0.9
                         # + dynamic safety margin (0.5 to 1.5 m)
```

**Altitude-Dependent Behavior:**
| Altitude | FOV Bound | Behavior |
|----------|-----------|----------|
| 3.0 m | ~1.8 m | Loose constraint, can chase at 1 m/s |
| 1.0 m | ~0.6 m | Tighter, moderate deceleration |
| 0.3 m | ~0.35 m | Very tight, strong deceleration |

---

## Flight Operations

### **Pre-Flight Checklist**

```
□ All 6 terminals active (check for [INFO] messages)
□ AprilTag detector running: /detections topic at 20 Hz
□ MPC solver receiving state: /mpc/state active
□ PX4 EKF2 converged: GPS fix, compass OK in QGC
□ Battery > 80%
□ Camera focus at infinity (focus=0.0 parameter set)
□ Lighting: Avoid direct sun glare on tags
□ Rover positioned: Landing pad in open area, flat ground
□ Wind: < 3 m/s for first tests
□ Kill-switch ready on RC
```

### **Launch Sequence (on RC or QGC)**

1. **Arm & Takeoff:**
   - Switch RC mode to "Offboard" or use QGC "Arm" button
   - Mission executor enters **SearchLandingPadMode** automatically
   - Drone climbs to `target_altitude` (4.0 m) and holds GPS position

2. **Search Phase:**
   - Switch to teh Position Mode or Vision Active Mode to manually move the drone above the landing pad.
   - When camera detects tag (` /landing/tag_visible_flag = true`), transitions to **VisionLandingMode**

3. **Vision Landing Phase:**
   - MPC loop active at 20 Hz
   - Drone tracks tag centroid using velocity commands
   - Adaptive decimation (2.0→1.0) triggers at 0.60 m altitude
   - Vertical descent rate: ~0.3 m/s (controlled by MPC cost function)

4. **Touchdown & Disarm:**
   - Below 0.1 m altitude, executor calls `land()` command
   - Pixhawk auto-levels and slowly touches down
   - Disarm on landing (0 throttle detected)
   - Executor waits for disarm confirmation → mission complete

### **Emergency Procedures**

| Failure Mode | Symptom | Recovery |
|---|---|---|
| **Tag Lost** | `/landing/tag_visible_flag = false` for > 0.5s | Reverts to GPS hold; pilot switches mode |
| **MPC Solver Crash** | Solver status ≠ "optimal" | Returns previous velocity command (safe failsafe) |
| **Camera Disconnected** | No `/detections` messages | Executor detects and logs warning |
| **Pixhawk Disconnected** | UDP packets stop | MicroXRCEAgent auto-disconnects; manual recovery |
| **Battery Low** | Battery % < `battery_failsafe_pct` | Executor calls RTB (Return-to-Base) |

**Manual Override:**
- Always available via RC stick movement (disables offboard mode)
- QGC "Switch Mode" button → Mode Manual (instant pilot control)

---

## Post-Flight Analysis

### **1. Extract & Process Flight Log**

```bash
# Copy CSV from Raspberry Pi to desktop
scp pi@192.168.1.241:/home/pi/px4_ros2_ws/hardware_analysis/thesis_landing_log_*.csv .

# Or find latest on Pi:
ssh pi@192.168.1.241
ls -ltr ~/px4_ros2_ws/hardware_analysis/ | tail -1
```

### **2. Automated Plot Generation**

```bash
# Run the plotting script (regenerates all 4 analysis plots)
python3 plot_fov_calibration.py

# Output: results/Plot_A_Visibility_vs_Altitude.png
#         results/Plot_B_PX4_vs_MoCap.png
#         results/Plot_C_Detection_Heatmap.png
#         results/Plot_D_Switching_Audit.png
```

### **3. Manual CSV Analysis (Python)**

```python
import pandas as pd
import numpy as np

# Load flight data
df = pd.read_csv('thesis_landing_log_1234567890.csv')

# Key metrics
print("=== LANDING ACCURACY ===")
# Compute touchdown point (last few rows below 0.1 m altitude)
landing_rows = df[df['mocap_drone_d'] >= 0.1]  # NED frame: d is down
if len(landing_rows) > 0:
    final_pos_n = landing_rows.iloc[-1]['mocap_drone_n']
    final_pos_e = landing_rows.iloc[-1]['mocap_drone_e']
    pad_n = df['mocap_pad_n'].median()  # Stable position
    pad_e = df['mocap_pad_e'].median()
    
    error_n = final_pos_n - pad_n
    error_e = final_pos_e - pad_e
    total_error = np.sqrt(error_n**2 + error_e**2)
    print(f"Horizontal error: {total_error:.3f} m ({total_error*100:.1f} cm)")

# Tag visibility timeline
print("\n=== TAG VISIBILITY ===")
print(f"Tag visible: {(df['tag_visible'] == 1).sum()} / {len(df)} frames")
print(f"  ({(df['tag_visible'] == 1).sum() / len(df) * 100:.1f}%)")

# MPC command amplitude
print("\n=== MPC CONTROL ===")
print(f"Max velocity command: {df['mpc_cmd_vn'].abs().max():.2f} m/s (N)")
print(f"Max velocity command: {df['mpc_cmd_ve'].abs().max():.2f} m/s (E)")

# Tracking error over time
print("\n=== TRACKING ERROR ===")
print(f"Mean horizontal error: {df['total_horiz_error'].mean():.3f} m")
print(f"Max horizontal error: {df['total_horiz_error'].max():.3f} m")
print(f"Std horizontal error: {df['total_horiz_error'].std():.3f} m")
```

---
 
## Achieved Metrics (Phase II: Static)
 
### Landing Accuracy
```
Final Touchdown Accuracy:  5.82 cm (horizontal)
Mean Tracking Error:       5.39 cm (in-flight)
```
 
### Target Detection & Handoff
```
Outer Bundle (400mm, 150mm tags):  87.9% detection rate (>0.6m altitude)
Tag 19 Precision (50mm tag):       12.1% active usage (<0.6m altitude)
Switching Threshold:               0.6m altitude (adaptive)
```
 
### Real-Time Performance
```
MPC Control Rate:          20 Hz (50 ms loop)
AprilTag Detection:        ~20 Hz (18–35 ms latency)
PX4 State Update:          50 Hz
CSV Logging:               30 Hz
```
 
### Embedded System Load
```
CPU Usage (Raspberry Pi 5):  <45%
Memory Usage:                ~200 MB
Detection Latency:           ~18–35 ms (decimation-dependent)
```
 
---

## Troubleshooting Guide

### **Camera Not Publishing Images**

```bash
# Check camera availability
v4l2-ctl --list-devices

# Verify camera info is published
ros2 topic echo /camera/camera_info --once

# If missing, check camera_node parameters
ros2 param dump camera_node
```

**Fix:** Ensure `camera:="/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a"` matches your device path.

---

### **AprilTag Detector Not Running**

```bash
# Check if image_rect is being published
ros2 topic hz /camera/image_rect

# Check apriltag_node status
ros2 node list | grep apriltag

# View detector logs
ros2 launch px4_vision_hardware_cpp landing.launch.py 2>&1 | grep -i apriltag
```

**Fix:** Ensure `image_proc` package is installed: `sudo apt install ros-humble-image-proc`

---

### **MPC Solver Not Receiving State**

```bash
# Check if /mpc/state is being published
ros2 topic echo /mpc/state

# Check if vision_landing_mode is active
ros2 node list | grep vision_landing

# View solver logs
ros2 node info mpc_solver_node
```

**Fix:** Ensure vision_landing_mode publishes to `/mpc/state` at 20 Hz (check `vision_landing_mode.cpp` line ~250).

---

### **Pixhawk Not Connecting (MicroXRCE)**

```bash
# Check DDS middleware
echo $FASTDDS_DEFAULT_PROFILES_FILE

# Manually test UDP connectivity
nc -u -l 8888 &  # Listen on UDP 8888
# From another terminal:
echo "test" | nc -u localhost 8888

# Restart MicroXRCEAgent
sudo pkill -f MicroXRCEAgent
sudo MicroXRCEAgent udp4 -p 8888 -v 6
```

**Fix:** If still no connection, check Pixhawk USB-C is fully seated and IP routing is correct (check `route -n` on Pi).

---

### **Landing Director Switching Logic Broken**

If drone doesn't switch from outer bundle to Tag 19 near touchdown:

```bash
# Enable debug output in landing_director.cpp:
# Uncomment RCLCPP_INFO_THROTTLE lines around line 110

# Check altitude being received
ros2 topic echo /fmu/out/vehicle_odometry --once | grep -A3 position

# Verify tag visibility flag
ros2 topic echo /landing/tag_visible_flag
```

**Expected behavior:**
- Above 0.60 m: `tracking_small_tag = false` (using outer bundle)
- Below 0.60 m + Tag 19 visible: `tracking_small_tag = true` (using Tag 19)
- Above 0.80 m: Revert to bundle

---

## Build & Compilation

### **From Source**

```bash
# Inside Docker on Raspberry Pi
cd ~/px4_ros2_ws

# Full build
colcon build --packages-select px4_vision_hardware_cpp \
             --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or incremental build (faster)
colcon build --packages-select px4_vision_hardware_cpp \
             --cmake-args -DCMAKE_BUILD_TYPE=Release \
             --symlink-install

# Build with verbose output
colcon build --packages-select px4_vision_hardware_cpp \
             --event-handlers console_direct+
```

### **Troubleshooting Compilation Errors**

```bash
# If apriltag headers not found:
sudo apt install ros-humble-apriltag-ros

# If px4_ros2 not found:
git clone https://github.com/px4/px4-ros2-interface-lib.git \
  ~/px4_ros2_ws/src/px4-ros2
colcon build --packages-select px4-ros2

# Clean build (if corrupted)
rm -rf build/ install/ log/
colcon build --packages-select px4_vision_hardware_cpp
```

---
 
## Validation Scope
 
### ✅ What WAS Tested
- [x] Centimeter-level precision landing (5.82 cm)
- [x] Multi-scale AprilTag detection reliability (87.9% bundle rate)
- [x] Real-time MPC solver on embedded platform (12.6% CPU load)
- [x] Adaptive tag switching logic (from 400mm→50mm bundle)
- [x] Vision-based trajectory tracking (5.39 cm mean error)
- [x] PX4-ROS 2 integration via px4_ros2 interface
- [x] Stationary landing pad precision
- [x] Full autonomous mission execution (search→track→land→disarm)
### ❌ What WAS NOT Tested (Out of Thesis Scope)
- [ ] **Moving target (rover velocity = 0 m/s in Phase II)**
- [ ] Dynamic relative motion (UAV-UGV closure rate)
- [ ] Wind speeds > 3 m/s
- [ ] Indoor/low-light conditions (daylight outdoor only)
- [ ] Non-flat landing surfaces (grass, obstacles)
- [ ] Multi-drone scenarios
- [ ] GPS-denied environments (vision-only flight)

---

## Author

Priyam Gupta [![LinkedIn](https://img.shields.io/badge/LinkedIn-%230077B5.svg?logo=linkedin&logoColor=white)](https://www.linkedin.com/in/priyam-gupta-5777b3190/)

[![GitHub](https://img.shields.io/badge/GitHub-09priyamgupta-black?logo=github)](https://github.com/09priyamgupta)

---

## License & Attribution

This work is part of the **Erasmus Mundus Joint Master's in Intelligent Field Robotic Systems (IFRoS)**, conducted at:
- **Saxion University of Applied Sciences** (Supervisors: Dr. ir. Abeje Mersha, Camille Buschman)
- **ELTE Budapest** (Supervisor: Dr. Zoltán Istenes)

---

**Last Updated:** June 2026 | **Thesis Defense:** Early July 2026 | **Status:** Hardware-Validated ✓