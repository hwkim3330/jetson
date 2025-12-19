#!/bin/bash
# Sync web files to nginx

SRC_DIR=/home/nvidia/ros2_ws/src/robot_web/www
DEST_DIR=/var/www/robot

if [ -d "$SRC_DIR" ]; then
    echo 'nvidia' | sudo -S mkdir -p $DEST_DIR 2>/dev/null
    echo 'nvidia' | sudo -S cp -r $SRC_DIR/* $DEST_DIR/ 2>/dev/null
    echo 'nvidia' | sudo -S chown -R www-data:www-data $DEST_DIR 2>/dev/null
    echo "Web files synced to $DEST_DIR"
else
    echo "Source directory not found: $SRC_DIR"
    exit 1
fi
