# Usage: 
# 	

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	# args that can be set from the command line or default will be used
	namespace_arg = DeclareLaunchArgument(
		"namespace", 
		default_value="anafi", 
		description="Namespace for this Anafi")
	
	anafi_gui_node = Node(
		package='anafi_gui',
		namespace=LaunchConfiguration('namespace'),
		executable='anafi_gui',
		name='anafi_gui',
		output="screen",
		emulate_tty=True,
		arguments=['--ros-args', '--log-level', 'INFO']
	)
        
	return LaunchDescription([
		namespace_arg,
		anafi_gui_node
	])
