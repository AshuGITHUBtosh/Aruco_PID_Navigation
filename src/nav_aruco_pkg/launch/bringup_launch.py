from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='nav_aruco_pkg',
            executable='bringup_launch',
            name='nav_aruco_pkg',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        '/home/go4av05/build_ws/src/nav_aruco_pkg/config/nav2_params.yaml']),
    ])
