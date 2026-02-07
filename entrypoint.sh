cd /sub
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /sub/install/setup.bash
ros2 launch sub_rsp launch_bot.launch.py