
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
            parameters=[
                {'models': ["CADs/new_41_73e89d0210597565.stl", "CADs/new_41_41e1ed7b94f48fd2.stl", "CADs/new_41_11308407c209d669.stl", "CADs/new_41_3038caf1166093b5.stl", "CADs/new_41_87121ce6219c550b.stl"]},
                {'wind_files': 'windsim/new_41_56781af51c0a888a'},
                {'free_point': '1.0,2.0,3.0'},
                {'cell_size': 0.1},
                {'output_directory': dir}
            ],
            output='log'
        ),
    ])