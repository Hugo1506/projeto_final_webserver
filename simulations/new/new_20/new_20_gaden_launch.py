
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
                {'models': ["CADs/new_20_a60c5e350b02ce33.dae", "CADs/new_20_aac6babe8c9c4eb0.stl"]},
                {'wind_files': 'windsim/new_20_9a658eca0f192e92.csv'},
                {'free_point': 'x,y,z'},
                {'cell_size': 0.1},
            ]
        ),
    ])
