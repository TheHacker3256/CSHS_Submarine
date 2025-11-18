from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
  joy_params = os.path.join(get_package_share_directory('sub_rsp'),'config','joystick.yaml')

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

  drone_cont_stamper = Node(
    package='twist_stamper',
    executable='twist_stamper',
    name = 'left_twist_stamper_node',
    arguments=['--ros-args',
     '-r',
     'cmd_vel_in:=drone_cont/reference_unstamped',
     '-r',
     'cmd_vel_out:=drone_cont/reference']
  )

  diffdrive_teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name = 'diffdrive_teleop_node',
    parameters=[joy_params],
    remappings=[('/cmd_vel', '/diffdrive_cont/cmd_vel_unstamped')]
  )

  diffdrive_stamper = Node(
    package='twist_stamper',
    executable='twist_stamper',
    name = 'left_twist_stamper_node',
    arguments=['--ros-args',
     '-r',
     'cmd_vel_in:=/diffdrive_cont/cmd_vel_unstamped',
     '-r',
     'cmd_vel_out:=/diffdrive_cont/cmd_vel']
  )


  twist_mux = Node(
    package='twist_mux',
    executable='twist_mux',
    name = 'twist_mux_node',
    parameters=[joy_params],
  )

  return LaunchDescription([
    joy_node,
    # twist_mux,
    diffdrive_teleop_node,
    drone_cont_teleop_node,
    diffdrive_stamper,
    drone_cont_stamper,
  ])
