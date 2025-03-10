from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    dir  = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        Node(
            package='gaden_preprocessing',
            executable='preprocessing',
            name='gaden_preprocessing_node',
            output='screen',
            parameters=[
                {'models': ["CADs/new_28_8d663191b14fae02.stl", "CADs/new_28_48d759145769eb58.stl", "CADs/new_28_0f5a7834db5e1995.stl", "CADs/new_28_89e6e55443f13f9f.stl", "CADs/new_28_488385bd46d7f3d5.stl"]},
                {'wind_files': 'windsim/new_28_d5ba955046e53010'},
                {'free_point': '1.0,2.0,3.0'},
		        {'cell_size': 0.1},
                {'output_directory': dir}               
            ]
        ),
    ])
