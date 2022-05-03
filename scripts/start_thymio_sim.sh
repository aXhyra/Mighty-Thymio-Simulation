#!/bin/bash
source ~/dev_ws/install/setup.bash
ros2 launch thymio_simulation controller.launch.py thymio_name:=thymio0
