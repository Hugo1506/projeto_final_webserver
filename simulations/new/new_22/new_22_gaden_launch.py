
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gaden_preprocessing',
            executable='preprocessing',
            name='gaden_preprocessing_node',
            output='screen',
            parameters=[
                {'models': ["CADs/new_22_273cf0775522b7e3.stl"]},
                {'wind_files': 'windsim/new_22_0.csv'},
                {'free_point': '1.0,2.0,3.0'},
		{'cell_size': 0.1}
            ]
        ),
    ])
