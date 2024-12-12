#!/bin/bash

# Replace with your Ethernet interface name
ETH_NAME="enp2s0f0"

# Ensure the script is run with sudo
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root. Use sudo." 
   exit 1
fi

# Flush existing IP addresses
echo "Flushing IP addresses on $ETH_NAME..."
ip addr flush dev $ETH_NAME
sleep 3

# Display the interface state (should be DOWN)
echo "Checking interface state..."
ip addr show dev $ETH_NAME | grep -E "state DOWN" || echo "$ETH_NAME is not in state DOWN."
sleep 3

# Assign a static IP address
echo "Assigning static IP address 10.5.5.1/24 to $ETH_NAME..."
ip addr add 10.5.5.1/24 dev $ETH_NAME
sleep 3

# Bring up the interface
echo "Bringing up the $ETH_NAME interface.... Connect the sensor!!"
sleep 10
ip link set $ETH_NAME up
sleep 3

# Display the interface state (should be UP)
echo "Verifying interface state..."
ip addr show dev $ETH_NAME | grep -E "state UP" || echo "$ETH_NAME is not in state UP."
sleep 3

# Configure network connection to the sensor
echo "Setting up network connection to the sensor..."
dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i $ETH_NAME --bind-dynamic 
