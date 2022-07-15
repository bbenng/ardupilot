#!/bin/bash

# Usage: ./rosswarm.sh [count]
# Note: Ensure that the offboard ROS package has been built successfully.
# https://github.com/alvintps/offboard
# APM offboard mode ROS package: https://github.com/alvintps/apm_offboard_mode

# Variable declaration
count="$1"

echo "Initializing $count USV instances in ROS..."
for (( i=1; i<=$count; i++ ))
do
    echo "Starting Vehicle $i..."
    gnome-terminal --title="Vehicle $i" --tab -- bash -c "roslaunch apm_offboard_mode apm_single_usv.launch tgt_system:=$i"
    sleep 1
done