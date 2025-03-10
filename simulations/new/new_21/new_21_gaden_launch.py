
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
                {'models': ["CADs/new_21_4a74a47e9dd29431.stl"]},
                {'wind_files': 'windsim/new_21_ae5609456cdf9ea7'},
		        {'empty_point_x': 1.0},
                {'empty_point_y': 1.0},
                {'empty_point_z': 0.5},
                {'cell_size': 0.1}
            ]
        ),
    ])
