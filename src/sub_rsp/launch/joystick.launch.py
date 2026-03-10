from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
  joy_params = os.path.join(get_package_share_directory('sub_rsp'),'config','joystick.yaml')
  twist_mux_params = os.path.join(get_package_share_directory('sub_rsp'),'config','twist_mux.yaml')

  joy_node = Node(
    package='joy',
    executable='joy_node',
    parameters=[joy_params],
  )

  drone_cont_teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name = 'drone_cont_teleop_node',
    parameters=[joy_params],
    remappings=[('/cmd_vel', 'drone_cont/reference_unstamped')]
  )

  drone_twist_mux = Node(
    package="twist_mux",
    executable="twist_mux",
    name="drone_twist_mux",
    output='screen',
    parameters=[twist_mux_params, {'use_sim_time': True}],
    remappings={('cmd_vel_out', 'drone_cont/twist_mux')},
  )

  drone_cont_stamper = Node(
    package='twist_stamper',
    executable='twist_stamper',
    name = 'left_twist_stamper_node',
    arguments=['--ros-args',
     '-r',
     'cmd_vel_in:=drone_cont/twist_mux',
     '-r',
     'cmd_vel_out:=drone_cont/reference']
  )

  diffdrive_teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='diffdrive_teleop_node',
    parameters=[joy_params],
    remappings=[('/cmd_vel', '/diffdrive_cont/cmd_vel_unstamped')]
  )

  diffdrive_twist_mux = Node(
    package="twist_mux",
    executable="twist_mux",
    name="diffdrive_twist_mux",
    output='screen',
    parameters=[twist_mux_params, {'use_sim_time': True}],
    remappings={('/cmd_vel_out', '/diffdrive_cont/twist_mux')},
  )

  diffdrive_stamper = Node(
    package='twist_stamper',
    executable='twist_stamper',
    name = 'left_twist_stamper_node',
    arguments=['--ros-args',
     '-r',
     'cmd_vel_in:=/diffdrive_cont/twist_mux',
     '-r',
     'cmd_vel_out:=/diffdrive_cont/cmd_vel']
  )



  return LaunchDescription([
    joy_node,
    diffdrive_teleop_node,
    drone_cont_teleop_node,
    diffdrive_stamper,
    drone_cont_stamper,
    diffdrive_twist_mux,
    drone_twist_mux,
  ])
