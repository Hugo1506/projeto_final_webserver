
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
                {'models': ["CADs/new_40_021f7173e0283ac2.stl", "CADs/new_40_ec779eea2fc243fc.stl", "CADs/new_40_8bb4413016a6d88f.stl", "CADs/new_40_1a1e6d8526125a7f.stl", "CADs/new_40_090c48f9722d1cc4.stl"]},
                {'wind_files': 'windsim/new_40_239cd9ecc3f0b82b'},
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