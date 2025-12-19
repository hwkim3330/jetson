---
layout: default
title: Hardware Setup
---

# Hardware Setup

This guide covers the hardware components and connections for the SDV platform.

## Components

### Main Board: NVIDIA Jetson Orin Nano Super

| Specification | Value |
|---------------|-------|
| GPU | 1024-core NVIDIA Ampere |
| CPU | 6-core Arm Cortex-A78AE |
| Memory | 8GB LPDDR5 |
| Storage | NVMe SSD (recommended) |
| AI Performance | 67 TOPS |

### Camera: IMX219 CSI

| Specification | Value |
|---------------|-------|
| Sensor | Sony IMX219 |
| Resolution | Up to 3280x2464 |
| Interface | MIPI CSI-2 |
| FOV | 160° (with wide-angle lens) |

**Connection**: CSI connector on Jetson carrier board

### LiDAR: LD19

| Specification | Value |
|---------------|-------|
| Range | 0.1m - 12m |
| Scan Rate | 10Hz |
| Angular Resolution | 1° |
| FOV | 360° |
| Interface | USB (UART) |

**Connection**: USB port → `/dev/ttyUSB0`

### Motor Driver

| Specification | Value |
|---------------|-------|
| Protocol | Modbus RTU |
| Interface | UART |
| Baud Rate | 115200 |

**Connection**: UART pins → `/dev/ttyTHS1`

## Wiring Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Jetson Orin Nano                         │
│                                                             │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐ │
│  │   CSI   │    │   USB   │    │  UART   │    │  Power  │ │
│  │ Camera  │    │  LiDAR  │    │  Motor  │    │   DC    │ │
│  └────┬────┘    └────┬────┘    └────┬────┘    └────┬────┘ │
│       │              │              │              │       │
└───────┼──────────────┼──────────────┼──────────────┼───────┘
        │              │              │              │
        ▼              ▼              ▼              ▼
   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐
   │ IMX219  │   │  LD19   │   │ Motor   │   │  12V    │
   │ Camera  │   │  LiDAR  │   │ Driver  │   │ Supply  │
   └─────────┘   └─────────┘   └─────────┘   └─────────┘
```

## Pin Connections

### UART (Motor Driver)

| Jetson Pin | Motor Driver |
|------------|--------------|
| Pin 8 (TXD) | RX |
| Pin 10 (RXD) | TX |
| Pin 6 (GND) | GND |

### Power

| Component | Voltage | Current |
|-----------|---------|---------|
| Jetson Orin Nano | 5V-19V DC | 3-5A |
| Motors | 12V DC | 2-4A |
| LiDAR | 5V (USB) | 0.5A |

## Device Configuration

### Check Connected Devices

```bash
# Check USB devices
lsusb

# Check serial ports
ls -la /dev/ttyUSB* /dev/ttyTHS*

# Check camera
ls /dev/video*
```

### Set Device Permissions

```bash
# Add user to dialout group (for serial ports)
sudo usermod -a -G dialout $USER

# Set permissions for UART
sudo chmod 666 /dev/ttyTHS1
```

### Create udev Rules

For persistent device names:

```bash
# LiDAR rule
sudo tee /etc/udev/rules.d/99-lidar.rules << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Camera Setup

### Check Camera Detection

```bash
# List video devices
v4l2-ctl --list-devices

# Test camera
gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=640,height=480' ! nvvidconv ! xvimagesink
```

### Camera Parameters

The camera is configured in `robot.launch.py`:

```python
parameters=[{
    'width': 640,
    'height': 360,
    'fps': 15,
    'quality': 80
}]
```

## LiDAR Setup

### Check LiDAR Connection

```bash
# Check USB serial
ls -la /dev/ttyUSB0

# Test communication
cat /dev/ttyUSB0
```

### LiDAR Parameters

Configured in `robot.launch.py`:

```python
parameters=[{
    'product_name': 'LDLiDAR_LD19',
    'topic_name': 'scan',
    'port_name': '/dev/ttyUSB0',
    'frame_id': 'base_laser'
}]
```

## Motor Driver Setup

### Check UART Connection

```bash
# Check UART port
ls -la /dev/ttyTHS1
```

### Motor Parameters

Configured in `robot_driver`:

```python
# Modbus RTU settings
SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 115200
```

## Troubleshooting

### Camera Issues

| Problem | Solution |
|---------|----------|
| No /dev/video0 | Check CSI cable connection |
| Black image | Check lens cap, cable |
| Low FPS | Reduce resolution |

### LiDAR Issues

| Problem | Solution |
|---------|----------|
| No /dev/ttyUSB0 | Check USB connection |
| No scan data | Check baud rate, permissions |
| Incomplete scan | Clean LiDAR lens |

### Motor Issues

| Problem | Solution |
|---------|----------|
| No response | Check wiring, power |
| Wrong direction | Swap motor wires |
| Jerky motion | Check Modbus address |

---

[Back to Home](index.md) | [Previous: Installation](installation.md) | [Next: Web UI](web-ui.md)
