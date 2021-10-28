# Запись облаков точек фильтрованных
rosrun pcl_ros pointcloud_to_pcd input:=/copter/filtered_pointcloud _prefix:=/home/pi/fly_log/pcd_filtered/clover_f_pc_

# Запись облаков точек безопасной зоны посадки
rosrun pcl_ros pointcloud_to_pcd input:=/copter/slz_place _prefix:=/home/pi/fly_log/pcd_slz/clover_slz_

# Запись логов найденных координат
rostopic echo -p /copter/slz_coordinates > /home/pi/fly_log/slz_coordinates.csv

# Запись bag-файла для ROS с данными пакета
rosbag record /copter/filtered_pointcloud /copter/slz_place /copter/slz_coordinates
