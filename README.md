# Holonomic-Controller-for-3-Wheeled-Vehicle

This repository implements a **3-wheel holonomic drive controller** and a full **autonomous navigation pipeline** in **ROS 2**.  
The system enables both manual and autonomous control, real-time mapping using **SLAM Toolbox**, and point-to-point navigation with **Nav2**.

---

## Overview

The project integrates custom ROS 2 nodes for holonomic drive control, odometry publishing, mapping, and navigation:
1. **cmd_vel_to_wheels** – converts `/cmd_vel` into individual wheel velocities.  
2. **omni_odometry** – estimates robot pose (`/odom`) from wheel encoders.  
3. **robot_localization** – optionally fuses IMU + odometry data via an EKF.  
4. **slam_toolbox** – performs 2D SLAM and builds occupancy grid maps.  
5. **Nav2** – handles global path planning, local obstacle avoidance, and navigation goals.

---
##  Nodes & Responsibilities

###  `cmd_vel_to_wheels` — Three-Wheel Controller
**Inputs:** `/cmd_vel` → `geometry_msgs/Twist`  
**Outputs:** wheel velocity commands (e.g., `/wheel_i/cmd_vel` or joint controllers)

**Functionality:**  
Converts `vx`, `vy`, and `wz` (Twist) into per-wheel velocities using the standard 3-wheel holonomic kinematic matrix.  
Implements velocity limits and acceleration (slew) limiting for smooth motion control.

 **Source:**  
[cmd_vel_to_wheels.py](https://github.com/prathameshdv/Holonomic-Drive-Control-and-Autonomous-SLAM/blob/main/src/cmd_vel_to_wheels.py)

---

### `omni_odometry`
**Inputs:** wheel encoder ticks or wheel velocities  
**Outputs:** `/odom` → `nav_msgs/Odometry`

**Functionality:**  
Performs dead-reckoning to estimate pose and twist from wheel rotations via inverse kinematics.  
Publishes the `/odom` topic and broadcasts the TF transform `odom → base_link`.  
Includes optional covariance estimation for integration with EKF.

---

---

### `slam_toolbox` — Online Mapping
**Inputs:** `/scan` (`sensor_msgs/LaserScan`), `/odom` (`nav_msgs/Odometry`), TF (`odom → base_link`)  
**Outputs:** `/map` (`nav_msgs/OccupancyGrid`), 

**Functionality:**  
Performs real-time mapping using LiDAR data and odometry.  
Includes pose-graph optimization, loop closure detection, and map serialization.  
Configured via YAML to specify map frame, update frequency, and optimization parameters.

---

### 🤖 `Nav2` — Autonomous Navigation Stack
**Inputs:** `/map`, `/amcl_pose` (or fused `/odom`), `/scan` (for costmaps)  
**Outputs:** `/cmd_vel` (used by `cmd_vel_to_wheels` for execution)

**Functionality:**  
Implements full navigation:
- Global planning 
- Local obstacle avoidance 
- Dynamic re-planning and recovery behaviors  
- Publishes safe velocity commands to `/cmd_vel`.

---
