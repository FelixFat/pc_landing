#include <iostream>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <pc_landing/Landing.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/kdtree/kdtree.h>

#define PI 3.14159265

float k = 0;

struct landing
{
    float x;
    float y;
    float z;
    float R;
};

struct frame
{
    int w;
    int h;
};

std::vector<landing> lp;
landing land = {0.0, 0.0, 0.0, 0.0};

landing landing_point(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    int gw = rand() % cloud.width;
    int gh = rand() % cloud.height;
    
    bool state = true;
    bool flag = false;
    int R = 1;
    int step = 1;
    
    std::vector<frame> po;
    
    float x, y, z;
    
    int goal_w, goal_h, goal_R = 0;
    
    while (true)
    {        
        for (int h = gh - R; h <= gh + R; h += step)
        {
            for (int w = gw - R; w <= gw + R; w += step)
            {
                if (pow(w - gw, 2) + pow(h - gh, 2) > pow(R, 2))
                    continue;
                
                if (w < 0 or w > cloud.width-1 or h < 0 or h > cloud.height-1)
                {
                    frame p = { gw, gh };
                    po.push_back(p);
                    
                    gw -= round((w - gw)/R);
                    gh -= round((h - gh)/R);
                    
                    state = false;
                    break;
                }
                
                x = cloud.at(w, h).x;
                y = cloud.at(w, h).y;
                z = cloud.at(w, h).z;
                if (x == 0 and y == 0 and z == 0)
                {
                    frame p;
                    p.w = gw;
                    p.h = gh;
                    po.push_back(p);
                    
                    gw -= round((w - gw)/R);
                    gh -= round((h - gh)/R);
                    
                    state = false;
                    break;
                }
            }
            if (!state)
                break;
        }
        
        if (state)
        {
            goal_w = gw;
            goal_h = gh;
            goal_R = R;
            
            R += step;
            po.clear();
        }
        else
        {
            state = true;
            for (auto p: po)
            {
                if (gw == p.w and gh == p.h)
                {
                    flag = true;
                    break;
                }
            }
        }
        
        if (flag)
            break;
    }
    
    landing circle = {0.0, 0.0, 0.0, 0.0};
    
    if (goal_R > 0)
    {
        float Radius = abs(cloud.at(goal_w, goal_h).x - cloud.at(goal_w + goal_R, goal_h).x);
    
        circle = { cloud.at(goal_w, goal_h).x, cloud.at(goal_w, goal_h).y, cloud.at(goal_w, goal_h).z, Radius };
    }
    
    return(circle);
}

pcl::PointCloud<pcl::PointXYZ> inliers_points(pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ> cloud)
{
    cloud.width = input->width;
    cloud.height = input->height;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    
    for (auto i: inliers.indices)
    {
        int w = trunc(i / cloud.width);
        int h = i % cloud.width;
        cloud.at(w, h) = input->at(w, h);
    }
    
    return(cloud);
}

// Функция обработки входного облака точек
void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input, *msg);
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Создание объекта сегментации методом RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> plane;
    plane.setOptimizeCoefficients(true);
    plane.setModelType(pcl::SACMODEL_PLANE);
    plane.setMethodType(pcl::SAC_RANSAC);
    plane.setMaxIterations(1000);
    plane.setDistanceThreshold(0.01);
    
    while (true)
    {
        plane.setInputCloud(msg);
        plane.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < 0.3 * input->size())
        {
            PCL_ERROR("Could not estimate a PLANAR model for the given dataset.\n");
            break;
        }
            
        cloud = inliers_points(*inliers, msg, cloud);

        // Извлечение найденных точек из общего облака
        for (auto i: inliers->indices)
        {
            int w = trunc(i / msg->width);
            int h = i % msg->width;
            msg->at(w, h).x = -1 + (((float) rand()) / (float) RAND_MAX) * 2;
            msg->at(w, h).y = -1 + (((float) rand()) / (float) RAND_MAX) * 2;
            msg->at(w, h).z = -100 + (((float) rand()) / (float) RAND_MAX) * 200;
        }

        // Создание KdTree объекта для поиска метода изолирования
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));
        tree->setInputCloud(ptr_cloud);

        // Параметры кластеризации
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(0.3 * cloud.size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(ptr_cloud);
        ec.extract(cluster_indices);

        // Оценка наклона
        float normal[3] = { 0, 0, 1 };
        float normal_place[3] = { coefficients->values[0], coefficients->values[1], coefficients->values[2] };
        float dot = normal[0]*normal_place[0] + normal[1]*normal_place[1] + normal[2]*normal_place[2];
        float len_normal = sqrt(pow(normal[0], 2) + pow(normal[1], 2) + pow(normal[2], 2));
        float len_normal_place = sqrt(pow(normal_place[0], 2) + pow(normal_place[1], 2) + pow(normal_place[2], 2));
        float angle = abs(acos(dot/(len_normal*len_normal_place)) * 180.0/PI);

        if (angle <= 20.0)
        {
            // Кластеризация
            for (auto i: cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
                
                cloud_cluster = inliers_points(i, msg, cloud_cluster);
                
                land = landing_point(cloud_cluster);
                
                if (PI * pow(land.R, 2) >= PI * pow(0.2, 2))
                    lp.push_back(land);
            }
        }
    }
    
    if (!lp.empty())
    {
        float temp = 0;
        for (auto p: lp)
            if (p.R > temp)
                land = p;
    }
}

// Главная функция
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc-searching-place");
    ros::NodeHandle n;
    
    ros::Publisher pub = n.advertise<pc_landing::Landing>("/copter/point_landing", 1);
    ros::Subscriber sub = n.subscribe("/copter/camera/pointcloud", 1, callback);
    
    pc_landing::Landing msg;
    msg.x = land.x;
    msg.y = land.y;
    msg.z = land.z;
    msg.R = land.R;
    pub.publish(msg);
    
    ROS_INFO("x:%f, y:%f, z:%f, R:%f", msg.x, msg.y, msg.z, msg.R);
    
    ros::spin();

    return 0;
}
