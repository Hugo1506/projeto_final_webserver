
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
                  {'model_0': 'simulations/new/new_14/CADs/new_14_a4956dbad3f998bc.dae'},
                  {'model_1': 'simulations/new/new_14/CADs/new_14_4ce1bdacc8cf27d5.stl'},
                  {'wind_files': 'simulations/new/new_14/windsim/new_14_92cb2c15a5ff9e3c.csv'},
                  {'free_point': 'x,y,z'},
                  {'cell_size': 0.1},
              ]
          ),
      ])
  