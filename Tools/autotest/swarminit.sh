!/bin/bash

# Instructions:
#     1. Change the AP_Path and QGC_Path to your ardupilot repository's Rover directory and your QGroundControl Application Image path
#     2. Next, save the modified script under the name "swarminit.sh" (Note: do NOT overwrite the default script)
# Usage: swarminit.sh [count]

# Variable declaration
location='PandanReservoir'
AP_PATH='/home/ra-lwj-apollo/ardupilot/Rover'
QGC_PATH='/home/ra-lwj-apollo/Documents/QGroundControl.AppImage'
count="${1:-1}"

echo "Initializing $count USV instances in SITL..."
for (( i=1; i<=$count; i++ ))
do
    echo "Starting Vehicle $i..."
    gnome-terminal --title="Vehicle $i" --tab -- bash -c "cd $AP_PATH;sim_vehicle.py -L $location -v Rover -f usv --sysid $i -I $i -Z swarminit.txt"
    sleep 0.1
done

echo "Starting QGroundControl..."
${QGC_PATH}
