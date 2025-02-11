from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
import os
import time

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'asbathama_bringup', 'udemy.launch.xml'],
            output='screen'
        ),
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', '-u', '/home/ashutosh/Desktop/build_ws-main/src/asbu_eyes/asbu_eyes/open_eyes.py'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['python3', '-u', '/home/ashutosh/Desktop/build_ws-main/src/asbu_eyes/asbu_eyes/receive_frame.py'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['python3', '-u', '/home/ashutosh/Desktop/build_ws-main/src/asbu_eyes/asbu_eyes/pid.py'],
                    output='screen'
                ),
            ]
        )
    ])
