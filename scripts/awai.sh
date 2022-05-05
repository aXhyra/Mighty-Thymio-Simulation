#!/bin/bash
source ~/.bashrc
source ~/dev_ws/install/setup.bash

echo "Start coppeliasim, load scene awai.ttt, start simulation and press enter"
read
echo "Start bridge with: bash scripts/start_bridge.sh single:=True and press enter"
read
echo "Starting simulation"
ros2 launch thymio_simulation awai.launch.py thymio_name:=thymio0
