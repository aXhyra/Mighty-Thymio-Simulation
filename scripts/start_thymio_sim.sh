#!/bin/bash
source ~/dev_ws/install/setup.bash
ros2 launch thymio_simulation bonus.launch.py thymio_name:=thymio_0 thymio1_name:=thymio_1 thymio2_name:=thymio_2 thymio3_name:=thymio_3
