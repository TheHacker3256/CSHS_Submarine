import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command

import xacro
def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')


    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('sub_rsp'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    #config = xacro.process_file(xacro_file).toxml()
    config = Command(['xacro ', xacro_file, ' sim_mode:=', sim_mode])

    # Create a robot_state_publisher node
    params = {'robot_description': config, 'use_sim_time': use_sim_time}



    return LaunchDescription([
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[params])
    ])