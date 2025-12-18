#!/bin/bash
# Sync web files to nginx

SRC_DIR=/home/nvidia/ros2_ws/src/robot_web/www
DEST_DIR=/var/www/robot

if [ -d "$SRC_DIR" ]; then
    sudo mkdir -p $DEST_DIR
    sudo cp -r $SRC_DIR/* $DEST_DIR/
    sudo chown -R www-data:www-data $DEST_DIR
    echo "Web files synced to $DEST_DIR"
else
    echo "Source directory not found: $SRC_DIR"
    exit 1
fi
