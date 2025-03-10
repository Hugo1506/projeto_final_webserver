
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
                {'models': ["CADs/new_25_f3b0b5047e9e2876.stl"]},
                {'wind_files': 'windsim/wind_at_cell_centers_0.csv'},
                {'free_point': '1.0,2.0,3.0'},
		{'cell_size': 0.1}
            ]
        ),
    ])
