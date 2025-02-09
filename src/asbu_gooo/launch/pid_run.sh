#!/bin/bash

# Open a new terminal and run the first command
gnome-terminal -- bash -c "ros2 launch asbathama_bringup udemy.launch.xml; exec bash"

# Wait for 15 seconds
sleep 15

# Open a new terminal and run the second command
gnome-terminal -- bash -c "python -u /home/go4av05/build_ws/src/asbu_eyes/asbu_eyes/open_eyes.py; exec bash"

# Open a new terminal and run the third command
gnome-terminal -- bash -c "python -u /home/go4av05/build_ws/src/asbu_eyes/asbu_eyes/receive_frame.py; exec bash"

# Open a new terminal and run the fourth command
gnome-terminal -- bash -c "python -u /home/go4av05/build_ws/src/asbu_eyes/asbu_eyes/pid.py; exec bash"
