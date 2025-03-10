
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
                {'models': ["CADs/new_19_a5632cb06eb5826d.dae", "CADs/new_19_39058541d6bbace3.stl", "CADs/new_19_1cc77d1dd9021e89.dae", "CADs/new_19_bfa6a4c05b3c8d72.stl", "CADs/new_19_98fd9017a7691962.dae", "CADs/new_19_f257515cc4504626.stl", "CADs/new_19_79a5989f48921905.dae", "CADs/new_19_a71387efc7ea9cc9.stl", "CADs/new_19_079e2542453b339b.dae", "CADs/new_19_e92d31b13349d965.stl"]},
                {'wind_files': 'windsim/new_19_d94a668e770a6790.csv'},
                {'empty_point_x': 0.0,
                    'empty_point_y': 0.0,
                    'empty_point_z': 0.0,},
                {'cell_size': 0.1},
                {'output_path': 'simulations/new/new_19'}
            ]
        ),
    ])
