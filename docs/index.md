---
layout: default
title: SDV - Software Defined Vehicle
---

<div align="center">
<h1>SDV Robot Documentation</h1>
<p><strong>NVIDIA Jetson Orin Nano 기반 자율주행 로봇 플랫폼</strong></p>

<p>
<a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS2"></a>
<a href="https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit"><img src="https://img.shields.io/badge/Jetson-Orin_Nano-76B900?logo=nvidia" alt="Jetson"></a>
<a href="https://www.robotis.com/"><img src="https://img.shields.io/badge/ROBOTIS-Dynamixel-red" alt="Dynamixel"></a>
</p>
</div>

---

## Overview

SDV(Software Defined Vehicle)는 웹 브라우저로 제어하는 자율주행 로봇입니다.

### Key Features

| Feature | Description |
|---------|-------------|
| **Web Control** | 모바일/PC 브라우저에서 실시간 제어 |
| **SLAM Mapping** | Cartographer 기반 실시간 맵 생성 |
| **Navigation** | Nav2 기반 자율 주행 |
| **Auto Explore** | LiDAR 기반 자동 맵 탐색 |
| **AI Detection** | YOLOv8 TensorRT 147 FPS |
| **Pan/Zoom Map** | 터치 제스처로 맵 확대/이동 |

---

## Hardware

| Component | Model |
|-----------|-------|
| **Main Board** | Jetson Orin Nano Super (8GB) |
| **Camera** | IMX219 CSI (160° FOV) |
| **LiDAR** | LD19 360° (0.1-12m) |
| **Motors** | ROBOTIS Dynamixel XL430 × 2 |
| **Controller** | OpenCR 1.0 |
| **Power** | DC 9V |

---

## Quick Start

### 1. Access Web UI

```
http://<ROBOT_IP>:8888
```

### 2. Controls

| Tab | Function |
|-----|----------|
| **Control** | Camera + Joystick |
| **Map** | SLAM / Navigation |
| **AI** | YOLO Detection |

### 3. Keyboard

| Key | Action |
|-----|--------|
| W/S | Forward/Backward |
| A/D | Turn Left/Right |
| Space | Stop |

---

## Documentation

<div class="doc-grid">

### Getting Started
- [Installation Guide](installation) - 설치 및 빌드 방법
- [Hardware Setup](hardware) - 하드웨어 연결 및 설정

### User Guide
- [Web UI Guide](web-ui) - 웹 인터페이스 사용법
- [SLAM & Navigation](slam-navigation) - 맵 생성 및 자율주행
- [AI Features](ai-features) - YOLO 객체 인식

</div>

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Web Browser (port 8888)                       │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │   [Control]  [Map]  [AI]                                    ││
│  ├─────────────────────────────────────────────────────────────┤│
│  │          Camera / Map Canvas (Pan/Zoom)                     ││
│  ├─────────────────┬───────────────────────────────────────────┤│
│  │     LiDAR       │              Joystick                     ││
│  └─────────────────┴───────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
                              │ WebSocket
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 Humble                               │
│   rosbridge │ camera_node │ mode_controller │ yolo_detector     │
│   robot_driver │ LD19 LiDAR │ cartographer │ Nav2               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         Hardware                                 │
│  Jetson Orin Nano │ IMX219 │ LD19 │ OpenCR │ Dynamixel XL430   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Performance

| Metric | Value |
|--------|-------|
| YOLO FPS | 147 FPS (TensorRT FP16) |
| Camera | 640×360 @ 15fps |
| LiDAR | 10Hz, 450+ points |
| Map Resolution | 5cm/cell |

---

## Links

- [GitHub Repository](https://github.com/hwkim3330/jetson_robot)
- [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
- [NVIDIA Jetson](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [ROS2 Humble](https://docs.ros.org/en/humble/)

---

<div align="center">
<p><strong>Developed by <a href="https://www.keti.re.kr">KETI</a></strong></p>
<p>Korea Electronics Technology Institute</p>
</div>
