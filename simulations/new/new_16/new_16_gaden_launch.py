
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
                  {'model_0': 'simulations/new/new_16/CADs/new_16_789d2ce90680e3b2.dae'},
                  {'model_1': 'simulations/new/new_16/CADs/new_16_bef9ff18cc5032e4.stl'},
                  {'wind_files': 'simulations/new/new_16/windsim/new_16_b13b5164ac473024.csv'},
                  {'free_point': 'x,y,z'},
                  {'cell_size': 0.1},
              ]
          ),
      ])
  