#!/bin/bash

# Source ROS installation
source /opt/ros/humble/setup.bash

# Directory path to the ROS2 Workspace
ROS_WS="ros_workspace"
USER="panos"

sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev wlo1

# Source the workspace if it exists
if [ -f "/home/$USER/$ROS_WS/install/setup.bash" ]; then
    source "/home/$USER/$ROS_WS/install/setup.bash"
    echo "sourcing: /home/$USER/$ROS_WS/install/setup.bash"
fi

# Execute the command passed to the script
exec "$@"
