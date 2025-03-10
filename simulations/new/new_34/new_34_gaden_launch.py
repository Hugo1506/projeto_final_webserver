
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
                {'models': ["CADs/new_34_11607daa7d5bd879.stl", "CADs/new_34_76a6afd7771bafd5.stl", "CADs/new_34_2e51ea0ade028b9e.stl", "CADs/new_34_ecb93e5f7aecc5a3.stl", "CADs/new_34_371d20c8dec8b975.stl"]},
                {'wind_files': 'windsim+new_34_48abf3eba89efbdc'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 