#!/usr/bin/bash

# Set Contianer Name
CONTAINER_NAME="ros2_container"

echo "Building ROS2-Humble Container"
docker build --rm --progress plain --no-cache -t $CONTAINER_NAME:latest .

echo "Docker Build Completed"
