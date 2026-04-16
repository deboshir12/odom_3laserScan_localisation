# odom_3laserScan_localisation

Для запуска необзодимо поместить пакет в рабочую дирректорию ROS2, а затем выполнить следующие команды:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_pkg display.launch.py
