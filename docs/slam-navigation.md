---
layout: default
title: SLAM & Navigation
---

# SLAM & Navigation

This guide covers the mapping and navigation features of the SDV platform.

## Overview

SDV uses two main components:
- **Cartographer**: For SLAM (Simultaneous Localization and Mapping)
- **Nav2**: For autonomous navigation

## SLAM (Mapping)

### Starting SLAM

1. Go to **Map** tab in Web UI
2. Press **SLAM** button
3. Drive the robot around using joystick
4. Watch the map build in real-time

### How It Works

```
LiDAR Scan → Cartographer → OccupancyGrid → Web Canvas
   │              │              │
   ▼              ▼              ▼
/scan        Processing       /map
(360°)       & Loop Closure   (2D Grid)
```

### Cartographer Configuration

Location: `src/robot_slam/config/robot_lds_2d.lua`

Key parameters:
```lua
-- Frame configuration
tracking_frame = "base_link"
published_frame = "odom"
provide_odom_frame = false

-- Sensor configuration
use_odometry = true
use_imu_data = false  -- No IMU on this robot

-- LiDAR settings
min_range = 0.1
max_range = 8.0

-- Map resolution
resolution = 0.05  -- 5cm per cell
```

### Saving Maps

1. After mapping, press **Save** button
2. Map is saved to `~/ros2_ws/maps/map.yaml`
3. Files created:
   - `map.yaml` - Metadata
   - `map.pgm` - Image file

### Map File Format

`map.yaml`:
```yaml
image: map.pgm
resolution: 0.050000
origin: [-7.07, -6.85, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## Navigation (Nav2)

### Starting Navigation

1. Ensure a map is saved
2. Press **Nav** button in Map tab
3. Wait for "Nav Active" status
4. Press **Goal** and tap destination on map

### How It Works

```
Goal → Global Planner → Local Planner → cmd_vel → Motors
 │          │                │              │
 ▼          ▼                ▼              ▼
Pose     A* Path         DWB Path      Velocity
Stamped  Planning       Following      Commands
```

### Navigation Flow

1. **Initial Pose**: Automatically set from odometry
2. **Goal Pose**: Set by tapping map
3. **Global Path**: Calculated by NavFn (blue line)
4. **Local Path**: Calculated by DWB (green line)
5. **Execution**: Robot follows path avoiding obstacles

### Nav2 Parameters

Location: `src/robot_navigation/param/humble/burger.yaml`

Key parameters:
```yaml
# AMCL (Localization)
amcl:
  robot_model_type: "differential"
  base_frame_id: "base_link"
  set_initial_pose: true
  initial_pose:
    x: 0.0
    y: 0.0
    yaw: 0.0

# Controller
controller_server:
  max_vel_x: 0.26
  max_vel_theta: 1.0
  min_vel_x: -0.26

# Global Costmap
global_costmap:
  robot_radius: 0.12

# Local Costmap
local_costmap:
  robot_radius: 0.12
```

### Path Visualization

| Path | Color | Description |
|------|-------|-------------|
| Global | Blue | Full route to goal |
| Local | Green | Immediate path (dashed) |

## Auto Exploration (Clean Mode)

### Overview

The Clean tab provides automatic map exploration, similar to a robot vacuum cleaner.

### Algorithm

```python
# 8-sector LiDAR analysis
# Find most open direction
# Move toward unexplored areas

if front_blocked:
    turn_to_open_space()
else:
    move_forward_with_steering()
```

### Parameters

```python
linear_speed = 0.2   # Forward speed (m/s)
angular_speed = 0.8  # Turn speed (rad/s)
min_distance = 0.4   # Obstacle threshold (m)
scan_sectors = 8     # Number of sectors
```

### Using Auto Explore

1. Go to **Clean** tab
2. Press **Auto SLAM** (green button)
3. Robot explores automatically
4. Watch coverage percentage increase
5. Press **Stop** when done
6. Go to Map tab and **Save**

## Topics Reference

### SLAM Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | OccupancyGrid | Generated map |
| `/scan` | LaserScan | LiDAR data |
| `/odom` | Odometry | Robot odometry |

### Navigation Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | PoseStamped | Navigation goal |
| `/initialpose` | PoseWithCovarianceStamped | Robot pose |
| `/plan` | Path | Global path |
| `/local_plan` | Path | Local path |

### Control Topics

| Topic | Message | Values |
|-------|---------|--------|
| `/robot/mode` | String | `manual`, `slam`, `nav`, `explore` |
| `/robot/save_map` | String | Any message triggers save |

## Command Line Usage

### Start SLAM Manually

```bash
ros2 launch robot_slam cartographer.launch.py
```

### Start Navigation Manually

```bash
ros2 launch robot_navigation navigation2.launch.py
```

### Save Map via CLI

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/map
```

### Set Initial Pose via CLI

```bash
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

### Send Goal via CLI

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position: {x: 1.0, y: 1.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

## Troubleshooting

### Map Not Building

- Check LiDAR is publishing: `ros2 topic hz /scan`
- Check odometry: `ros2 topic echo /odom`
- Verify SLAM is running: `ros2 node list | grep cartographer`

### Navigation Not Working

- Ensure map is saved
- Check AMCL is localized
- Verify TF tree: `ros2 run tf2_tools view_frames`

### Robot Not Moving to Goal

- Check goal is reachable (not through walls)
- Verify costmaps are configured
- Check controller is running

### Path Not Generated

- Verify map→odom TF exists
- Check global costmap is loaded
- Ensure goal is in free space

---

[Back to Home](index.md) | [Previous: Web UI](web-ui.md) | [Next: AI Features](ai-features.md)
