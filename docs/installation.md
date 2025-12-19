---
layout: default
title: Installation Guide
---

# Installation Guide

This guide covers the complete installation process for the SDV platform.

## Prerequisites

### Hardware Requirements

- NVIDIA Jetson Orin Nano Super (8GB)
- IMX219 CSI Camera
- LD19 360° LiDAR
- Modbus Motor Driver
- DC 12V Power Supply

### Software Requirements

- JetPack 6.x
- ROS2 Humble
- TensorRT 10.x
- Python 3.10+

## Step 1: ROS2 Humble Installation

If ROS2 Humble is not installed:

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Install additional packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-cartographer ros-humble-cartographer-ros ros-humble-rosbridge-server -y
```

## Step 2: Clone Repository

```bash
cd ~
git clone https://github.com/hwkim3330/jetson_robot.git ros2_ws
cd ~/ros2_ws
```

## Step 3: Install Dependencies

```bash
# Install rosdep
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

# Install package dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Install Python packages
pip3 install ultralytics numpy opencv-python
```

## Step 4: Build Workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
```

## Step 5: Environment Setup

Add to `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export KETI_MODEL=robot" >> ~/.bashrc
echo "export LDS_MODEL=LDS-04" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc
```

## Step 6: Web UI Setup

### Install nginx

```bash
sudo apt install nginx -y
```

### Create web directory

```bash
sudo mkdir -p /var/www/robot
sudo cp -r ~/ros2_ws/src/robot_web/www/* /var/www/robot/
sudo chown -R www-data:www-data /var/www/robot
```

### Configure nginx

```bash
sudo tee /etc/nginx/sites-available/robot << 'EOF'
server {
    listen 8888;
    server_name _;
    root /var/www/robot;
    index index.html;

    location / {
        try_files $uri $uri/ =404;
    }
}
EOF

sudo ln -sf /etc/nginx/sites-available/robot /etc/nginx/sites-enabled/
sudo rm -f /etc/nginx/sites-enabled/default
sudo nginx -t && sudo systemctl restart nginx
sudo systemctl enable nginx
```

## Step 7: Systemd Service Setup

### Create start script

```bash
mkdir -p ~/ros2_ws/scripts
cat > ~/ros2_ws/scripts/start_robot.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export KETI_MODEL=robot
export LDS_MODEL=LDS-04
export ROS_DOMAIN_ID=30

# Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 3

# Start mode controller
ros2 run robot_ai mode_controller.py &
sleep 1

# Start robot
ros2 launch robot_bringup robot.launch.py
EOF

chmod +x ~/ros2_ws/scripts/start_robot.sh
```

### Create systemd service

```bash
sudo tee /etc/systemd/system/robot.service << 'EOF'
[Unit]
Description=SDV Robot Service
After=network.target

[Service]
Type=simple
User=nvidia
WorkingDirectory=/home/nvidia/ros2_ws
ExecStart=/home/nvidia/ros2_ws/scripts/start_robot.sh
Restart=on-failure
RestartSec=10
Environment="HOME=/home/nvidia"

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl start robot.service
```

## Step 8: TensorRT Model Setup (Optional)

For AI features, build the TensorRT engine:

```bash
cd ~/ros2_ws/src/robot_ai/models

# Download YOLOv8n model
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# Export to ONNX
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx')"

# Build TensorRT engine (FP16)
/usr/src/tensorrt/bin/trtexec --onnx=yolov8n.onnx --saveEngine=yolov8n_fp16.engine --fp16
```

## Verification

### Check services

```bash
systemctl status robot.service nginx
```

### Check ROS2 nodes

```bash
source ~/ros2_ws/install/setup.bash
ros2 node list
```

### Access Web UI

Open `http://<ROBOT_IP>:8888` in your browser.

## Troubleshooting

### Build fails

```bash
# Clean and rebuild
rm -rf build install log
colcon build --symlink-install
```

### Service won't start

```bash
# Check logs
journalctl -u robot.service -f
```

### Camera not working

```bash
# Check CSI connection
ls /dev/video*
```

### LiDAR not working

```bash
# Check USB connection
ls /dev/ttyUSB*
```

---

[Back to Home](index.md) | [Next: Hardware Setup](hardware.md)
