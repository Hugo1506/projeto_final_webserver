
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
                {'models': ['simulations/new/new_18/CADs/new_18_8e5d384a794ec42e.dae', 'simulations/new/new_18/CADs/new_18_97f6ad8efd7600bf.stl', 'simulations/new/new_18/CADs/new_18_c7c673fb6aa03ea9.dae', 'simulations/new/new_18/CADs/new_18_6c8690e7f60af6c2.stl', 'simulations/new/new_18/CADs/new_18_e04d386ef16900e5.dae', 'simulations/new/new_18/CADs/new_18_f1045cd967d44b4c.stl', 'simulations/new/new_18/CADs/new_18_b764580f476d5342.dae', 'simulations/new/new_18/CADs/new_18_5131b63e29dd45bd.stl', 'simulations/new/new_18/CADs/new_18_21860191870a8526.dae', 'simulations/new/new_18/CADs/new_18_07206bd6e26b2fff.stl']},
                {'wind_files': 'simulations/new/new_18/windsim/new_18_60ce34091ec0e13c.csv'},
                {'free_point': 'x,y,z'},
                {'cell_size': 0.1},
            ]
        ),
    ])
