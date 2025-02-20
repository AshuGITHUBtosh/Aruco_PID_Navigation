## Overview
This project implements an autonomous navigation system using ROS2 and OpenCV, enabling real-time tracking of ArUco markers for precise navigation. It incorporates PID control to ensure accurate alignment of the robot with detected markers.
## Tools used
 - ROS2 Humble
 - OpenCV
 - PID control
 - Python

## Dependencies & Installation
### Install ROS2 on Ubuntu
- ```sudo apt install ros-humble-desktop```
- ```source /opt/ros/humble/setup.bash```
### Install Python Dependencies
- ```pip install opencv-python numpy scipy```

### Install ROS2 Packages
- ```sudo apt update && sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge python3-numpy```

 ## Execution
 - ``` colcon build ```
 - ``` source install/setup.bash```
 - ``` ros2 launch asbu_gooo asbathama_follow.launch.py```

   
 
