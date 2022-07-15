!/bin/bash

# Usage: ./swarminit.sh [count]

# Variable declaration
location='PandanReservoir'
AP_PATH='/home/ra-lwj-apollo/ardupilot/Rover'
QGC_PATH='/home/ra-lwj-apollo/Documents/QGroundControl.AppImage'
count="$1"

echo "Initializing $count USV instances in SITL..."
for (( i=1; i<=$count; i++ ))
do
    echo "Starting Vehicle $i..."
    gnome-terminal --title="Vehicle $i" --tab -- bash -c "cd $AP_PATH;sim_vehicle.py -L $location -v Rover -f usv --sysid $i -I $i -Z swarminit.txt"
    sleep 0.1
done

echo "Starting QGroundControl..."
${QGC_PATH}
