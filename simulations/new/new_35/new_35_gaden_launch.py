
from launch import LaunchDescription
from launch_ros.actions import Node
import os

dir  = os.path.dirname(os.path.realpath(__file__))

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gaden_preprocessing',
            executable='preprocessing',
            name='gaden_preprocessing_node',
            output='screen',
            parameters=[
                {'models': ["CADs/new_35_56a937abd606cdaf.stl", "CADs/new_35_a0668f6b8dc1b13c.stl", "CADs/new_35_025fb76816835fce.stl", "CADs/new_35_84c9e909d30dbabc.stl", "CADs/new_35_632418deabbf3a13.stl"]},
                {'wind_files': 'windsim/new_35_d79842c16326ab48'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 