#include <ros/ros.h>
#include <pc_landing/OrganizePointCloud.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

bool organize(pc_landing::OrganizePointCloud::Request  &req, pc_landing::OrganizePointCloud::Response &res)
{
    return(true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "organize");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/copter/organized_pc", organize);
    
    ros::spinOnce();

    return 0;
} 
