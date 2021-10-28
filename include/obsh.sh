# Запуск пакета:
roslaunch pc_landing search.launch path:=/home/pi

# Запись облака точек PointCloud2 с камеры во время полета в PCD:
rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth_registered/points _prefix:=/home/pi/fly_log/pcd/clover_

# Запись сообщений о точках посадки:
rostopic echo -p /copter/slz_coordinates > /home/pi/fly_log/slz_coordinates.csv

# Запись bag-файла:
rosbag record -a
