
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
                {'models': ["CADs/new_38_6fd308c302ce846a.stl", "CADs/new_38_3b8332c9ebd102b6.stl", "CADs/new_38_65f0c7f3c4974154.stl", "CADs/new_38_17af5caf396155ce.stl", "CADs/new_38_ef0129f7940e3dd2.stl"]},
                {'wind_files': 'windsim/new_38_90d8f9c69b43e04c'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 