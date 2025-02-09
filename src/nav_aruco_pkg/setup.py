from setuptools import setup
import os
from glob import glob

package_name = 'nav_aruco_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/bringup_launch.py']),],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='go4av05',
    maintainer_email='go4av05.pvtx@gmail.com',
    description='Goal Navigator and ArUco Architecture Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intermediate_goal_setter = nav_aruco_pkg.intermediate_goal_setter:main',
            'aruco_detector = nav_aruco_pkg.aruco_detector:main',
            'rover_pose_publisher = nav_aruco_pkg.rover_pose_publisher:main',
            'final_goal_node = nav_aruco_pkg.final_goal_node:main',
        ],
    },
)
