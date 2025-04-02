from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


	return LaunchDescription([ 
	        Node(
	            package='pc2_to_grid',
	            executable='3dmap',
	            name='pc2map_saver',
	            output='screen',
	            parameters=[
	                {"pc2_topic": "/bonxai_point_cloud_centers"},
	                {"map_name": "/home/orin/my_3dmap.png"},
                    {"rotation_angle": 45.0},
	            ]
	        )
	    ])