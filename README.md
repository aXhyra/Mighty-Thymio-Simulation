# Homework assignment 2
## Alind Xhyra, Federico Pallotti
## Robotics, May 2022

# Links to videos and scenes

* [Simulation videos](https://drive.switch.ch/index.php/s/A28TJbkiR2SMahK)
* [Coppeliasim Scenes](https://drive.switch.ch/index.php/s/85PkQwTrLL9wkTV)


# How to launch Task 1
To launch the first task:
* Open coppeliasim
* Load scene 8.ttt from the scenes folder
* Start simulation
* Launch bridge with `bash scripts/start_bridge.sh`
* Run `source ~/dev_ws/install/setup.bash`
* Start simulation with `ros2 launch thymio_simulation task_1.launch.py thymio_name:=thymio_0`

## Video of task 1
[![](https://drive.switch.ch/index.php/apps/files_sharing/ajax/publicpreview.php?x=3840&y=1314&a=true&file=Task_1_snapshot.png&t=RWxrYFFnNv1ojOs&scalingup=0)](https://drive.switch.ch/index.php/s/F720Uj3vgeKUcn8)

# How to launch Task 2
To launch the second task:
* Open coppeliasim
* Load scene wall.ttt from the scenes folder
* Start simulation
* Launch bridge with `bash scripts/start_bridge.sh`
* Run `source ~/dev_ws/install/setup.bash`
* Start simulation with `ros2 launch thymio_simulation task_2.launch.py thymio_name:=thymio_0`

## Video of task 2
[![](https://drive.switch.ch/index.php/apps/files_sharing/ajax/publicpreview.php?x=3840&y=1314&a=true&file=Task_2_snapshot.png&t=NTnPDr0xPNv9NQu&scalingup=0)](https://drive.switch.ch/index.php/s/PGYY6H9vdnslFxo)

# How to launch Task 3
To launch the third (compulsory) task:
* Open coppeliasim
* Load scene wall.ttt from the scenes folder
* Start simulation
* Launch bridge with `bash scripts/start_bridge.sh`
* Run `source ~/dev_ws/install/setup.bash`
* Start simulation with `ros2 launch thymio_simulation compulsory.launch.py thymio_name:=thymio_0`

## Video of task 3
[![](https://drive.switch.ch/index.php/apps/files_sharing/ajax/publicpreview.php?x=3840&y=1314&a=true&file=Task_3_snapshot.png&t=J1gKO072kQgO6sQ&scalingup=0)](https://drive.switch.ch/index.php/s/bJp1fba89kH54Ou)

# How to launch Bonus 1
To launch the first bonus task:
* Open coppeliasim
* Load scene awai.ttt from the scenes folder
* Start simulation
* Launch bridge with `bash scripts/start_bridge.sh`
* Run `source ~/dev_ws/install/setup.bash`
* Start simulation with `ros2 launch thymio_simulation awai.launch.py thymio_name:=thymio_0`

## Bonus 1 video
[![](https://drive.switch.ch/index.php/apps/files_sharing/ajax/publicpreview.php?x=3840&y=1314&a=true&file=Bonus_1_snapshot.png&t=UWyR4vO1mTjj5ET&scalingup=0)](https://drive.switch.ch/index.php/s/qDz1FYfn8ADLmdR)

# How to launch Bonus 2
To launch the second bonus task:
* Open coppeliasim
* Load scene bonus.ttt from the scenes folder
* Start simulation
* Launch bridge with `bash scripts/start_bridge.sh single:=False`
* Run `source ~/dev_ws/install/setup.bash`
* Start simulation with `ros2 launch thymio_simulation bonus.launch.py thymio_name:=thymio_0 thymio1_name:=thymio_1 thymio2_name:=thymio_2 thymio3_name:=thymio_3`

## Info
The launch file for this task is set to handle up to 4 thymios, adding more will result in only 4 thymios being launched.

## Bonus 2 video
[![](https://drive.switch.ch/index.php/apps/files_sharing/ajax/publicpreview.php?x=3840&y=1314&a=true&file=Bonus_2_snapshot.png&t=5Ooamo4WmTMWXTk&scalingup=0)](https://drive.switch.ch/index.php/s/fjd19jmIipBpbse)

# Other info
Noise data from thymio's proximity sensors has been handled by using a rolling average of the last 3 measurements.
Also data from the proximity sensor has been normalized in a range from 0 to 1 and inverted in such a way that the closer the object is, the higher the value of the sensor.

The scripts folder containes scripts to quickly source and run the launch files for each point.
