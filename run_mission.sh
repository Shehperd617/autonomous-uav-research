#!/bin/bash
echo "Cleaning up old processes..."
pkill -9 -f arducopter
pkill -9 -f sim_vehicle
pkill -9 -f mavproxy
sleep 2

echo "Launching SITL with High-Speed Freestyle Physics (No Middleman)..."
# --no-mavproxy prevents the matplotlib crash.
sim_vehicle.py -v ArduCopter --frame freestyle --add-param-file=$HOME/autonomous-uav-research/speed.parm --no-mavproxy -w > $HOME/autonomous-uav-research/sitl.log 2>&1 &

echo "Waiting 10 seconds for SITL to initialize..."
sleep 10

echo "Launching The Spear..."
# Connecting directly to the raw C++ binary port
python3 $HOME/autonomous-uav-research/interceptor_v11.py --connect tcp:127.0.0.1:5760
