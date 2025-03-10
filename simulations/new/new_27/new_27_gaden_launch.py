
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
                {'models': ["CADs/new_27_eacc99af5c52bbb7.stl"]},
                {'wind_files': 'windsim/new_27_c7048f334d9eacc6'},
                {'free_point': '1.0,2.0,3.0'},
		{'cell_size': 0.1}
            ]
        ),
    ])
