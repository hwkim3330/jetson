#!/bin/bash
# KETI Robot Auto-Start Script
# For systemd service - runs all robot nodes
# Web UI is served by nginx (port 8888)

set -e

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash

# Set environment variables
export KETI_MODEL=robot
export LDS_MODEL=LDS-04
export ROS_DOMAIN_ID=30
export HOME=/home/nvidia
export USER=nvidia

# Log file
LOG_DIR=/home/nvidia/ros2_ws/logs
mkdir -p $LOG_DIR
LOG_FILE=$LOG_DIR/robot_$(date +%Y%m%d_%H%M%S).log

log() {
    echo "[$(date '+%H:%M:%S')] $1" | tee -a $LOG_FILE
}

cleanup() {
    log "Shutting down..."
    pkill -f "yolo_detector" 2>/dev/null || true
    pkill -f "gesture_detector" 2>/dev/null || true
    pkill -f "rosbridge" 2>/dev/null || true
    pkill -f "robot.launch.py" 2>/dev/null || true
    exit 0
}

trap cleanup SIGTERM SIGINT

log "================================================"
log "Starting KETI Robot"
log "================================================"

# Kill any existing processes
pkill -f "yolo_detector" 2>/dev/null || true
pkill -f "gesture_detector" 2>/dev/null || true
pkill -f "rosbridge" 2>/dev/null || true
pkill -f "robot.launch.py" 2>/dev/null || true
sleep 2

# Start rosbridge for web interface (port 9090)
log "[1/2] Starting rosbridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
sleep 3

# Start robot bringup (includes camera_node, mode_controller)
# mode_controller will auto-start YOLO and Gesture detectors
log "[2/2] Starting robot_bringup..."
ros2 launch robot_bringup robot.launch.py 2>&1 | tee -a $LOG_FILE &
ROBOT_PID=$!

log "================================================"
log "All services started"
log "Web UI: http://$(hostname -I | awk '{print $1}'):8888"
log "Camera: http://$(hostname -I | awk '{print $1}'):8080/stream"
log "================================================"

# Wait for main process
wait $ROBOT_PID
