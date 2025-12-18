#!/bin/bash
# KETI Robot Stop Script

echo "Stopping KETI Robot..."

pkill -f "robot.launch.py" 2>/dev/null
pkill -f "rosbridge" 2>/dev/null
pkill -f "web_video_server" 2>/dev/null
pkill -f "web_server.py" 2>/dev/null
pkill -f "mode_controller" 2>/dev/null
pkill -f "rf2o_laser" 2>/dev/null
pkill -f "robot_ros" 2>/dev/null
pkill -f "hlds_laser" 2>/dev/null

sleep 1
echo "Robot stopped."
