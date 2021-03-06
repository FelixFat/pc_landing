#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <pc_landing/LandingCoordinates.h>
#include <pc_landing/LandingPoint.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/angles.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include "pc_landing.h"

#define PI 3.14159265

class PC_Search
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub_pc_;
    ros::Publisher pub_place_;
    ros::Subscriber sub_lp_;
    ros::Subscriber sub_dist_;
    ros::ServiceClient client_;
    
public:
    PC_Search()
    {
        pc_model_angle = 20.0;
        pc_square_min = 0.0;    //0.126
        pc_range_sensor = 1.0;
        pc_radius_m = 0.0;
        v_lp_mass.clear();
        pc_landing_area = { 0.0, 0.0, 0.0, 0.0 };
        
        pub_        = n_.advertise<pc_landing::LandingCoordinates>("/copter/slz_coordinates", 1);
        pub_pc_     = n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/copter/filtered_pointcloud", 1);
        pub_place_  = n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/copter/slz_place", 1);
        
        sub_dist_   = n_.subscribe("/rangeginder/range", 10, &PC_Search::callback_dist, this);
        sub_lp_     = n_.subscribe("/camera/depth_registered/points", 10, &PC_Search::callback_lp, this);
        client_     = n_.serviceClient<pc_landing::LandingPoint>("/copter/landing_point");
    }

    // Функция приведения входящих в область облака точек
    pcl::PointCloud<pcl::PointXYZ> inliers_points(pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        cloud->width = input->width;
        cloud->height = input->height;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);
        
        for (auto i: inliers.indices)
        {
            cloud->points[i].x = input->points[i].x;
            cloud->points[i].y = input->points[i].y;
            cloud->points[i].z = input->points[i].z;
        }
        
        return(*cloud);
    }

    // Функция приведения индексов облака точек
    pcl::PointIndices indexes(pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& msg)
    {
        pcl::PointIndices indexes;
        indexes.header = inliers.header;
        
        int count = 0;
        for (int i = 0; i < input->size(); i++)
        {
            if (input->points[i].x == msg->points[count].x and
                input->points[i].y == msg->points[count].y and
                input->points[i].z == msg->points[count].z
            )
            {
                indexes.indices.push_back(i);
                count++;
            }
        }
        
        return(indexes);
    }
    
    // Основная функция
    void callback_lp(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Конвертация типа для обработки
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_input (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *ptr_input);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr f_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
        
        // Расчет расстояния (приведение коэффициентов облака точек в метры)
        float pc_range = pc_range_sensor / ptr_input->at(ptr_input->height/2, ptr_input->width/2).z;
        
        // Расчет минимального числа точек для анализа облаков
        int pc_points_num_min = 0.3 * ptr_input->size();
        
        // Входное облако точек
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
        for (auto p: ptr_input->points)
        {
            if (isnan(p.x) or isnan(p.y) or isnan(p.z))
            {
                continue;
            }
            else
            {
                msg->push_back(p);
            }
        }
        
        // Настройка модели детектирования
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> plane;
        plane.setOptimizeCoefficients(true);
        plane.setModelType(pcl::SACMODEL_PLANE);
        plane.setMethodType(pcl::SAC_PROSAC);
        plane.setEpsAngle(pc_model_angle);
        plane.setDistanceThreshold(0.01);
        
        // Настройка модели сегментации
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        
        // Настройка модели выделение пригодных областей
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(pc_points_num_min);
        
        int size = 0;
        while (msg->size() > pc_points_num_min)
        {
            // Детектирование плоскости методом RANSAC
            plane.setInputCloud(msg);
            plane.segment(*inliers, *coefficients);
            
            if (inliers->indices.size() == 0)
            {
                ROS_INFO("Landing zone not detected!");
                break;
            }
            else
            {
                ROS_INFO("Landing zone is detected!");
            }
            
            // Извлечение ровных поверхностей
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(msg);
            extract.setIndices(inliers);
            
            extract.setNegative(false);
            extract.filter(*ptr_cloud);
            
            *output += *ptr_cloud;
            
            // Использование метода Kd-деревьев для сегментации областей
            tree->setInputCloud(ptr_cloud);
            
            // Выделение пригодных областей
            std::vector<pcl::PointIndices> cluster_indices;
            ec.setSearchMethod(tree);
            ec.setInputCloud(ptr_cloud);
            ec.extract(cluster_indices);
            
            // Кластеризация
            for(auto i: cluster_indices)
            {
                // Выделение кластеров в отдельные облака
                pcl::PointIndices ind = indexes(i, ptr_input, ptr_cloud);
                *ptr_cloud = inliers_points(ind, ptr_input, ptr_cloud);
                
                // Поиск точки посадки
                if (ptr_cloud->size() > size)
                {
                    *f_cloud = *ptr_cloud;
                    size = f_cloud->size();
                    
                    t_landing_circle temp_slz = { 0.0, 0.0, 0.0, 0.0 };
                    
                    pc_landing::LandingPoint srv;
                    
                    sensor_msgs::PointCloud2 cloud_msg;
                    pcl::toROSMsg(*f_cloud, cloud_msg);
                    
                    srv.request.input = cloud_msg;
                    if (client_.call(srv))
                    {
                        ROS_INFO("Searching landing point...");
                        temp_slz.x = srv.response.x;
                        temp_slz.y = srv.response.y;
                        temp_slz.z = srv.response.z;
                        temp_slz.R = srv.response.R;
                    }
                    else
                    {
                        ROS_ERROR("Service server don't work!");
                    }
                    
                    // Проверка точки на соответствие требованию площади
                    pc_radius_m = temp_slz.R * pc_range;
                    if (M_PI * pow(pc_radius_m, 2) >= pc_square_min)
                    {
                        pc_landing_area = temp_slz;
                    }
                }
            }
            
            extract.setNegative(true);
            extract.filter(*ptr_cloud);
            *msg = *ptr_cloud;
        }
        
        // Вывод координат точки посадки
        pc_landing::LandingCoordinates slz_lp;
        slz_lp.x = pc_landing_area.x;
        slz_lp.y = pc_landing_area.y;
        slz_lp.z = pc_landing_area.z;
        slz_lp.R = pc_landing_area.R;
        slz_lp.x_metres = pc_landing_area.x * pc_range;
        slz_lp.y_metres = pc_landing_area.y * pc_range;
        pub_.publish(slz_lp);
        
        //Вывод найденных облаков точек
        sensor_msgs::PointCloud2 output_msg;
        
        pcl::toROSMsg(*output, output_msg);
        output_msg.header.frame_id = "camera_link";
        pub_pc_.publish(output_msg);
        
        pcl::toROSMsg(*f_cloud, output_msg);
        output_msg.header.frame_id = "camera_link";
        pub_place_.publish(output_msg);
    }
    
    void callback_dist(const sensor_msgs::RangeConstPtr& input)
    {
        pc_range_sensor = input->range + 0.025;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_detection_slz");
    
    PC_Search space;
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        PC_Search space;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
