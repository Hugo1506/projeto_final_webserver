
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
                {'models': ["CADs/new_37_ba6cbc6a1ebd253e.stl", "CADs/new_37_3f77529a167d8430.stl", "CADs/new_37_144d14928beedf7a.stl", "CADs/new_37_046a0857d0b69d94.stl", "CADs/new_37_04eb7f64af25d72d.stl"]},
                {'wind_files': 'windsim/new_37_124a81a6d9fae069'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 