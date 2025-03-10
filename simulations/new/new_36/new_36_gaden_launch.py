
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
                {'models': ["CADs/new_36_92dc5e8602bb7944.dae", "CADs/new_36_7f27f8ffdbfc4936.stl", "CADs/new_36_22952f165940456f.dae", "CADs/new_36_ee17237bbe16b604.stl", "CADs/new_36_a744623651623943.dae", "CADs/new_36_d6526d000d67d2dd.stl", "CADs/new_36_b9a4b66e6a7930b2.dae", "CADs/new_36_4105ff896aedabfc.stl", "CADs/new_36_1acb260ee2445870.dae", "CADs/new_36_b33eb74e0a6d57dc.stl"]},
                {'wind_files': 'windsim/new_36_1f5938ae405d5d5d'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 