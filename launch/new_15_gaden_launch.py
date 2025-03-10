
  from launch import LaunchDescription
  from launch_ros.actions import Node
  
  def generate_launch_description():
      return LaunchDescription([
          Node(
              package='gaden_preprocessing',
              executable='gaden_preprocessing_node',
              name='gaden_preprocessing_node',
              output='screen',
              parameters=[
                  {'model_0': 'simulations/new/new_15/CADs/new_15_7c785c8a99695238.dae'},
                  {'model_1': 'simulations/new/new_15/CADs/new_15_00498ce221622252.stl'},
                  {'wind_files': 'simulations/new/new_15/windsim/new_15_c925cb2c0f5a87dc.csv'},
                  {'free_point': 'x,y,z'},
                  {'cell_size': 0.1},
              ]
          ),
      ])
  