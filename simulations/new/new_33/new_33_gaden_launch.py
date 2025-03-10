
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
                {'models': ["CADs/new_33_07fc76bce79e6376.stl", "CADs/new_33_185c5ebf90451adb.stl", "CADs/new_33_a8c090fa35d8fb94.stl", "CADs/new_33_28e602d129fc9bc3.stl", "CADs/new_33_17ae4e5c02643334.stl"]},
                {'wind_files': 'new_33_857ab78608715f87'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 