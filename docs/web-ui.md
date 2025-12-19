---
layout: default
title: Web UI Guide
---

# Web UI Guide

The SDV Web UI provides a mobile-friendly interface to control your robot from any browser.

## Accessing the UI

Open your browser and navigate to:

```
http://<ROBOT_IP>:8888
```

## Layout Overview

```
┌───────────────────────────────────────────────────────────────┐
│ [Logo] SDV      [Control] [Map] [Clean] [AI]        ● Online  │
├───────────────────────────────────────────────────────────────┤
│                                                               │
│                    Camera / Map View                          │
│                        (16:9)                                 │
│                                                               │
├─────────────────────────┬─────────────────────────────────────┤
│                         │                                     │
│        LiDAR            │           Joystick                  │
│        View             │           Control                   │
│                         │                                     │
├─────────────────────────┴─────────────────────────────────────┤
│              X: 0.00    Y: 0.00    θ: 0°                     │
└───────────────────────────────────────────────────────────────┘
```

## Tabs

### Control Tab

The main control interface:

- **Camera View**: Real-time 16:9 video stream
- **LiDAR View**: 360° scan visualization with distance colors
- **Joystick**: Touch/drag to control movement
- **Odometry**: X, Y position and orientation (θ)

### Map Tab

SLAM and navigation interface:

- **Map Canvas**: Real-time map visualization
- **SLAM Button**: Start/stop mapping
- **Save Button**: Save current map
- **Nav Button**: Start navigation mode
- **Goal Button**: Set navigation destination
- **Stop Button**: Emergency stop

**Workflow**:
1. Press **SLAM** → Drive around to create map
2. Press **Save** → Save map to disk
3. Press **Nav** → Load map and start navigation
4. Press **Goal** → Tap on map to set destination

### Clean Tab

Auto exploration mode (like a robot vacuum):

- **Auto SLAM Button**: Start automatic exploration
- **Stop Button**: Stop exploration
- **Coverage %**: Shows explored area percentage

The robot automatically explores the environment using LiDAR to find open spaces.

### AI Tab

Object detection interface:

- **Camera View**: Video with bounding boxes
- **YOLO Button**: Toggle object detection
- **Detection Log**: Shows detected objects

## Controls

### Joystick

- **Touch/Drag**: Control robot movement
- **Center**: Stop
- **Up**: Forward
- **Down**: Backward
- **Left/Right**: Turn

### Keyboard (Desktop)

| Key | Action |
|-----|--------|
| `W` / `w` | Forward |
| `S` / `s` | Backward |
| `A` / `a` | Turn Left |
| `D` / `d` | Turn Right |
| `Space` | Emergency Stop |

### Map Interaction

- **Tap**: Set goal position (in Nav mode)
- **Goal direction**: Automatically faces destination

## LiDAR Visualization

The LiDAR view shows a 360° scan with color-coded distances:

| Distance | Color |
|----------|-------|
| < 0.3m | Red |
| 0.3-0.6m | Orange |
| 0.6-1.0m | Yellow |
| > 1.0m | Green |

## Status Indicators

### Connection Status

| Indicator | Meaning |
|-----------|---------|
| Green dot | Connected |
| Red dot | Disconnected |

### Map Status

| Status | Meaning |
|--------|---------|
| Ready | Idle state |
| SLAM Active | Mapping in progress |
| Nav Active | Navigation ready |
| Navigating to (X, Y) | Moving to goal |

## Mobile Optimization

The UI is optimized for mobile browsers:

- Touch-friendly controls
- Responsive layout
- Works in portrait/landscape

## Troubleshooting

### Camera Not Loading

- Check if camera_node is running
- Verify port 8080 is accessible
- Refresh the page

### Joystick Not Working

- Check WebSocket connection (port 9090)
- Look for rosbridge errors in logs
- Refresh and reconnect

### Map Not Updating

- Press SLAM button to start mapping
- Drive the robot to generate scan data
- Check if `/map` topic is publishing

### No Connection

- Check robot.service status
- Verify IP address
- Check firewall settings

---

[Back to Home](index.md) | [Previous: Hardware](hardware.md) | [Next: SLAM & Navigation](slam-navigation.md)
