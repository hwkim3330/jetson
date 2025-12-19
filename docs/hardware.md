---
layout: default
title: Hardware Setup
---

# Hardware Setup

This guide covers the hardware components and connections for the SDV platform.

## Components Overview

| Component | Model | Interface |
|-----------|-------|-----------|
| Main Board | Jetson Orin Nano Super 8GB | - |
| Camera | IMX219 CSI (160° FOV) | MIPI CSI-2 |
| LiDAR | LD19 360° | USB (UART) |
| Motors | Dynamixel XL430-W250-T × 2 | TTL (3-pin) |
| Controller | OpenCR 1.0 | USB |
| Power | 9V DC (2S LiPo or Adapter) | DC Jack |

---

## Main Board: NVIDIA Jetson Orin Nano Super

| Specification | Value |
|---------------|-------|
| GPU | 1024-core NVIDIA Ampere |
| CPU | 6-core Arm Cortex-A78AE |
| Memory | 8GB LPDDR5 |
| Storage | NVMe SSD (recommended) |
| AI Performance | 67 TOPS (INT8) |
| Power | 7W ~ 25W |

---

## Motors: ROBOTIS Dynamixel XL430-W250-T

High-performance smart actuators with integrated controller.

| Specification | Value |
|---------------|-------|
| Model | XL430-W250-T |
| Stall Torque | 1.0 N·m (@ 9V) |
| No Load Speed | 47 rpm (@ 9V) |
| Operating Voltage | 6.5V ~ 12V |
| Protocol | Dynamixel Protocol 2.0 |
| Resolution | 4096 steps/rev |
| Feedback | Position, Speed, Load, Voltage, Temperature |
| Interface | TTL (3-pin: GND, VCC, Data) |

### Motor IDs

| Motor | ID | Position |
|-------|-----|----------|
| Left Wheel | 1 | Left side |
| Right Wheel | 2 | Right side |

---

## Controller: OpenCR 1.0

Embedded controller board for TurtleBot3 series.

| Specification | Value |
|---------------|-------|
| MCU | ARM Cortex-M7 (STM32F746) |
| Clock | 216 MHz |
| Flash | 1 MB |
| SRAM | 320 KB |
| Interface | USB, Dynamixel TTL |
| IMU | ICM-20648 (6-axis) |

### Connection

```
Jetson (USB) ──── OpenCR ──┬── Dynamixel (Left)
                           └── Dynamixel (Right)
```

**Device**: `/dev/ttyACM0` or `/dev/ttyTHS1`

---

## Camera: IMX219 CSI

Sony IMX219 sensor with wide-angle lens.

| Specification | Value |
|---------------|-------|
| Sensor | Sony IMX219 |
| Resolution | Up to 3280 × 2464 |
| Used Resolution | 640 × 360 (16:9) |
| FOV | 160° (with wide-angle lens) |
| Interface | MIPI CSI-2 |
| FPS | 15 (configured) |

**Connection**: CSI connector on Jetson carrier board

---

## LiDAR: LD19

360° 2D laser scanner for SLAM and obstacle detection.

| Specification | Value |
|---------------|-------|
| Range | 0.1m ~ 12m |
| Scan Rate | 10 Hz |
| Angular Resolution | ~1° |
| Points per Scan | 450+ |
| FOV | 360° |
| Interface | USB (UART, 230400 baud) |
| Frame ID | `base_laser` |

**Connection**: USB port → `/dev/ttyUSB0`

---

## Power System

| Component | Voltage | Current |
|-----------|---------|---------|
| Jetson Orin Nano | 9V ~ 19V DC | 3 ~ 5A |
| Dynamixel Motors | 9V DC | 0.5A × 2 |
| OpenCR | 9V DC (from motors) | 0.3A |
| LiDAR | 5V (USB powered) | 0.5A |
| **Total** | **9V DC** | **~5A** |

### Power Options

1. **2S LiPo Battery** (7.4V nominal, 8.4V full)
   - Capacity: 2000mAh+ recommended
   - Runtime: ~30-60 minutes

2. **DC Adapter** (9V, 5A)
   - For development/testing
   - Stable power supply

---

## Wiring Diagram

```
                    ┌─────────────────────────────────────┐
                    │         Jetson Orin Nano             │
                    │                                      │
                    │  ┌──────┐  ┌──────┐  ┌──────────┐  │
                    │  │ CSI  │  │ USB  │  │ USB/UART │  │
                    │  └──┬───┘  └──┬───┘  └────┬─────┘  │
                    │     │         │           │         │
                    └─────┼─────────┼───────────┼─────────┘
                          │         │           │
          ┌───────────────┘         │           └───────────────┐
          │                         │                           │
          ▼                         ▼                           ▼
    ┌──────────┐             ┌──────────┐               ┌──────────┐
    │  IMX219  │             │   LD19   │               │  OpenCR  │
    │  Camera  │             │  LiDAR   │               │   1.0    │
    │ (160°FOV)│             │  (360°)  │               │          │
    └──────────┘             └──────────┘               └────┬─────┘
                                                              │
                              ┌────────────────────────┬──────┘
                              │                        │
                              ▼                        ▼
                        ┌──────────┐            ┌──────────┐
                        │ XL430    │            │ XL430    │
                        │ (Left)   │            │ (Right)  │
                        │  ID: 1   │            │  ID: 2   │
                        └──────────┘            └──────────┘
                              │                        │
                              └──────────┬─────────────┘
                                         │
                                         ▼
                                   ┌──────────┐
                                   │   9V DC  │
                                   │  Power   │
                                   └──────────┘
```

---

## Robot Dimensions

| Parameter | Value |
|-----------|-------|
| Wheel Diameter | 66mm |
| Wheel Separation | 160mm |
| Robot Width | ~180mm |
| Robot Length | ~140mm |
| Robot Height | ~200mm |
| Weight | ~1.0 kg (without battery) |

---

## Device Configuration

### Check Connected Devices

```bash
# Check USB devices
lsusb

# Check serial ports
ls -la /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS*

# Check camera
ls /dev/video*
v4l2-ctl --list-devices
```

### Set Device Permissions

```bash
# Add user to dialout group (for serial ports)
sudo usermod -a -G dialout $USER

# Apply group changes (or reboot)
newgrp dialout
```

### Create udev Rules

For persistent device names:

```bash
# LiDAR rule (LD19)
sudo tee /etc/udev/rules.d/99-lidar.rules << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
EOF

# OpenCR rule
sudo tee /etc/udev/rules.d/99-opencr.rules << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="opencr", MODE="0666"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## Troubleshooting

### Camera Issues

| Problem | Solution |
|---------|----------|
| No /dev/video0 | Check CSI cable connection |
| Black image | Check lens cap, verify cable seated properly |
| Low FPS | Reduce resolution in config |
| Overheating | Add heatsink, improve ventilation |

### LiDAR Issues

| Problem | Solution |
|---------|----------|
| No /dev/ttyUSB0 | Check USB connection, try different port |
| No scan data | Verify baud rate (230400), check permissions |
| Incomplete scan | Clean LiDAR lens, check for obstructions |
| Noisy data | Check power supply stability |

### Motor Issues

| Problem | Solution |
|---------|----------|
| No response | Check power, verify OpenCR connection |
| Wrong direction | Check motor ID assignment |
| Jerky motion | Check voltage (should be ~9V) |
| Overheating | Reduce load, check for mechanical binding |
| LED error | Check Dynamixel status in ROBOTIS software |

### OpenCR Issues

| Problem | Solution |
|---------|----------|
| Not detected | Check USB cable, try different port |
| Firmware issue | Re-flash OpenCR firmware |
| IMU drift | Calibrate IMU, place on flat surface |

---

## Firmware Updates

### OpenCR Firmware

```bash
# Install OpenCR tools
sudo apt install ros-humble-turtlebot3-msgs ros-humble-dynamixel-sdk

# Update firmware (if needed)
ros2 run turtlebot3_bringup create_udev_rules
```

### Dynamixel Configuration

Use ROBOTIS software to configure motors:
- [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

---

[Back to Home](index.md) | [Previous: Installation](installation.md) | [Next: Web UI](web-ui.md)
