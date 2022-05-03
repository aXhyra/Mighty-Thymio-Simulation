#!/bin/bash
source ~/dev_ws/install/setup.bash
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0
