#!/bin/bash
source ~/.bashrc
source ~/dev_ws/install/setup.bash

echo "Start coppeliasim, load scene bonus.ttt, start simulation and press enter"
read
echo "Start bridge with: bash scripts/start_bridge.sh single:=False and press enter"
read
echo "Starting simulation"
ros2 launch thymio_simulation bonus.launch.py thymio_name:=thymio_0 thymio1_name:=thymio_1 thymio2_name:=thymio_2 thymio3_name:=thymio_3
