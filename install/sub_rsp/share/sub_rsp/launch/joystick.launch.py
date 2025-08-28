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

  right_teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name = 'right_teleop_node',
    parameters=[joy_params],
    remappings=[('/cmd_vel', 'drone_cont/reference_unstamped')]
  )
  
  left_teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name = 'left_teleop_node',
    parameters=[joy_params],
    remappings=[('/cmd_vel', '/diffdrive_cont/cmd_vel_unstamped')]
  )

  return LaunchDescription([
    joy_node,
    right_teleop_node,
    left_teleop_node
  ])
