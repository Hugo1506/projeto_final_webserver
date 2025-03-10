
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
            output='log',
            parameters=[
                {'models': ["CADs/new_39_5ec33948a6703143.stl", "CADs/new_39_a64623ac962e6a39.stl", "CADs/new_39_29fec0ec4347ee15.stl", "CADs/new_39_c2c205d0771ca1e6.stl", "CADs/new_39_fe253a04782eb5ac.stl"]},
                {'wind_files': 'windsim/new_39_14382ed71aeda64f'},
                {'free_point': '1.0,2.0,3.0'},
                {'cell_size': 0.1},
                {'output_directory': dir}
            ],
            # Log output to file
            output={
                'stdout': os.path.join(dir, 'simulation_output.log'),
                'stderr': os.path.join(dir, 'simulation_error.log')
            }
        ),
    ])