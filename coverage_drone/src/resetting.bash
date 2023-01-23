#!/bin/sh

gz model -m quadrotor
sleep 0.5
echo "Starting hover Drone"
rosrun coverage_drone hover_drone.py
rosparam set /marker_color b



