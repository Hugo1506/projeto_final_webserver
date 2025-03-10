
from launch import LaunchDescription
from launch_ros.actions import Node

dir  = os.path.dirname(os.path.realpath(__file__))

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gaden_preprocessing',
            executable='preprocessing',
            name='gaden_preprocessing_node',
            output='screen',
            parameters=[
                {'models': ["CADs/new_30_cf10eb65b26fa114.stl", "CADs/new_30_2d83bc95a4cc42de.stl", "CADs/new_30_64fba61e1b8157fd.stl", "CADs/new_30_e604fb0dfdd44b42.stl", "CADs/new_30_69ff8cdc803c092b.stl"]},
                {'wind_files': 'new_30_ed1875ba779bd86a',
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 