from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

	urdf_path = os.path.join(get_package_share_path('rover_description', 'urdf', 'asbathama.urdf'))

	return LaunchDescription([

	])