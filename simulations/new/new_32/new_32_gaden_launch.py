
from launch import LaunchDescription
from launch_ros.actions import node
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
                {'models': ["CADs/new_32_b8ae9b8f3b7886cd.stl", "CADs/new_32_d4f708b9f34ff03e.stl", "CADs/new_32_bf8088b31e8330ca.stl", "CADs/new_32_86e145a97b0dadb2.stl", "CADs/new_32_b85a8c1c2d23a8b3.stl"]},
                {'wind_files': 'new_32_fc533ee24d1ba86f'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 