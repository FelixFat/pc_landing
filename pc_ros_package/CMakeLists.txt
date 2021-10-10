cmake_minimum_required(VERSION 3.0.2)
project(pc_landing)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pcl_ros
    pcl_conversions
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    FILES
    LandingCoordinates.msg
)

add_service_files(
    FILES
    LandingPoint.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
#   INCLUDE_DIRS include
    LIBRARIES pc_landing
    CATKIN_DEPENDS roscpp rospy pcl_ros pcl_conversions std_msgs sensor_msgs message_runtime
#   DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

###########
#  Nodes  #
###########

add_executable(pc_detection_slz src/pc_detection_slz.cpp)
target_link_libraries(pc_detection_slz ${catkin_LIBRARIES})
add_dependencies(pc_detection_slz pc_landing_gencpp)			#FOR MELODIC!!!

##############
#  Services  #
##############

add_executable(pc_search_point src/pc_search_point.cpp)
target_link_libraries(pc_search_point ${catkin_LIBRARIES})
add_dependencies(pc_search_point pc_landing_gencpp)				#FOR MELODIC!!!
