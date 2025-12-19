---
layout: default
title: SDV - Software Defined Vehicle
---

# SDV Documentation

Welcome to the SDV (Software Defined Vehicle) documentation. This platform provides a complete autonomous robot solution based on NVIDIA Jetson Orin Nano.

## Quick Links

- [Installation Guide](installation.md)
- [Hardware Setup](hardware.md)
- [Web UI Guide](web-ui.md)
- [SLAM & Navigation](slam-navigation.md)
- [AI Features](ai-features.md)
- [API Reference](api-reference.md)

## System Overview

SDV is a web-controlled autonomous robot platform featuring:

- **Web-based Control**: Control your robot from any browser
- **SLAM Mapping**: Create maps using Cartographer
- **Autonomous Navigation**: Navigate using Nav2
- **Auto Exploration**: Automatic map building (like a robot vacuum)
- **AI Detection**: YOLOv8 object detection at 147 FPS

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Web Browser (port 8888)                       │
├─────────────────────────────────────────────────────────────────┤
│                        ROS2 Humble                               │
│   rosbridge │ camera_node │ mode_controller │ yolo_detector     │
├─────────────────────────────────────────────────────────────────┤
│                        Hardware                                  │
│   Jetson Orin Nano │ IMX219 Camera │ LD19 LiDAR │ Motors        │
└─────────────────────────────────────────────────────────────────┘
```

## Getting Started

### 1. Clone Repository

```bash
git clone https://github.com/hwkim3330/jetson_robot.git ~/ros2_ws
cd ~/ros2_ws
```

### 2. Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3. Start Services

```bash
sudo systemctl start robot.service nginx
```

### 4. Access Web UI

Open `http://<ROBOT_IP>:8888` in your browser.

## Tabs Overview

| Tab | Description |
|-----|-------------|
| **Control** | Camera view with joystick control |
| **Map** | SLAM mapping and navigation |
| **Clean** | Auto exploration mode |
| **AI** | YOLO object detection |

## Support

- [GitHub Issues](https://github.com/hwkim3330/jetson_robot/issues)
- [KETI](https://www.keti.re.kr/)
