#!/bin/bash

# Note: Ensure that the offboard ROS package has been built successfully.
# https://github.com/alvintps/offboard

# Variable declaration
count=2

echo "Initializing multiple USV instances in ROS..."
for (( i=1; i<=$count; i++ ))
do
    echo "Starting Vehicle $i..."
    gnome-terminal --title="Vehicle $i" --tab -- bash -c "roslaunch offboard apm_single_usv.launch tgt_system:=$i"
    sleep 1
done