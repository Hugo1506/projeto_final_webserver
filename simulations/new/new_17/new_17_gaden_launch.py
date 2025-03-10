
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
                {'model_0': 'simulations/new/new_17/CADs/new_17_9f635356e88a4960.dae'},
                {'model_1': 'simulations/new/new_17/CADs/new_17_94270c760ac48577.stl'},
                {'wind_files': 'simulations/new/new_17/windsim/new_17_f41dd56aca32f262.csv'},
                {'free_point': 'x,y,z'},
                {'cell_size': 0.1},
            ]
        ),
    ])
