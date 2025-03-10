
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
                {'models': ["CADs/new_29_0daa0929d1b2d21e.stl", "CADs/new_29_48b68ab092f15656.stl", "CADs/new_29_d40cd148b58d35f8.stl", "CADs/new_29_66a403f81d9eccfe.stl", "CADs/new_29_5eaccda5714b7d47.stl"]},
                {'wind_files': 'windsim/new_29_df68e655a3fe4d9b_0.csv'},
                {'free_point': '1.0,2.0,3.0'},
		{'cell_size': 0.1}
            ]
        ),
    ])
