import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
    package_name='sub_rsp'

    rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory(package_name),'launch','rsp.launch.py')]), 
      launch_arguments={'use_sim_time': 'true', 'sim_mode': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
      launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')}.items()
    )

    imu_visualiser = Node(
      package='imu_filter_madgwick',
      executable='imu_filter_madgwick_node',
      parameters=[{'use_mag': False, 'publish_tf': True, 'world_frame': 'enu'}],
      remappings=[("/imu/data_raw", "/imu/mpu6050")],
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
      package='gazebo_ros', 
      executable='spawn_entity.py',
      arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
      output='screen'
    )

    diff_drive_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["diffdrive_cont"],
    )

    drone_drive_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["drone_cont"],
    )

    joint_broad_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_broad"],
    )

    joy = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory(package_name),'launch','joystick.launch.py')])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            output='screen',
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings={('/cmd_vel_out', '/cmd_vel')},
        )
    
    # Launch them all!
    return LaunchDescription([
      rsp,
      #gazebo sim stuff
      gazebo,
      spawn_entity,
      #controller spawners
      joint_broad_spawner,
      diff_drive_spawner,
      drone_drive_spawner,
      #other nodes
      imu_visualiser,
      joy,
      twist_mux
    ])
