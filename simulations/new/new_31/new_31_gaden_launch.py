
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
                {'models': ["CADs/new_31_e9ef1daba9ee458c.stl", "CADs/new_31_91d8ae65c9d74a15.stl", "CADs/new_31_ca96b6d64723a987.stl", "CADs/new_31_803a8d84ff7d263b.stl", "CADs/new_31_c844b0d653fb56ed.stl"]},
                {'wind_files': 'new_31_983ceafdda9a947c'},
                {'free_point': '1.0,2.0,3.0'},
		            {'cell_size': 0.1},
                {'output_directory': dir}                 
            ]
        ),
    ]) 