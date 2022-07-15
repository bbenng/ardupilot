!/bin/bash

# Variable declaration
address='127.0.0.1'
location='PandanReservoir'
AP_PATH='/home/ra-lwj-apollo/ardupilot/Rover'
GCS_PATH='/home/ra-lwj-apollo/Documents/'
count=2

echo "Initializing multiple USV instances in SITL..."
for (( i=1; i<=$count; i++ ))
do
    echo "Starting Vehicle $i..."
    (( port=$i+8000 ))
    gnome-terminal --title="Vehicle $i" --tab -- bash -c "cd $AP_PATH;sim_vehicle.py -L $location -f usv --sysid $i -I $i --out=tcpin:$address:$port -Z swarminit.txt"
    sleep 0.1
done

echo "Starting QGroundControl..."
cd $GCS_PATH
./QGroundControl.AppImage
