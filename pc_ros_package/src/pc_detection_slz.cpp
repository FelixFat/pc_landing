#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
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
        sub_dist_   = n_.subscribe("/copter/dist_to_center", 1, &PC_Search::callback_dist, this);
        sub_lp_     = n_.subscribe("/camera/depth_registered/points", 10, &PC_Search::callback_lp, this);
        client_     = n_.serviceClient<pc_landing::LandingPoint>("/copter/landing_point");
    }

    // Функция приведения входящих в область облака точек
    pcl::PointCloud<pcl::PointXYZ> inliers_points(pcl::PointIndices::Ptr& inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ> cloud)
    {
        cloud.width = input->width;
        cloud.height = input->height;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);
        
        for (auto i: inliers->indices)
        {
            cloud.points[i].x = input->points[i].x;
            cloud.points[i].y = input->points[i].y;
            cloud.points[i].z = input->points[i].z;
        }
        
        return(cloud);
    }

    // Функция приведения индексов облака точек
    pcl::PointIndices indexes(pcl::PointIndices::Ptr& inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& msg)
    {
        pcl::PointIndices indexes;
        indexes.header = inliers->header;
        
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
        plane.setMethodType(pcl::SAC_RANSAC);
        plane.setEpsAngle(pc_model_angle);
        plane.setMaxIterations(1000);
        plane.setDistanceThreshold(0.01);
        
        // Настройка модели сегментации
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        
        // Настройка модели выделение пригодных областей
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(pc_points_num_min);
        
        while (true)
        {
            // Детектирование плоскости методом RANSAC
            plane.setInputCloud(msg);
            plane.segment(*inliers, *coefficients);
            
            if (inliers->indices.size() < pc_points_num_min)
            {
                ROS_INFO("Landing zone not detected!");
                break;
            }
            else
            {
                ROS_INFO("Landing zone is detected!");
            }
            
            // Выделение детектированных областей в отдельное облако
            pcl::PointIndices ind = indexes(inliers, ptr_input, msg);
            pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices(ind));
            
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud = inliers_points(new_inliers, ptr_input, cloud);
            
            // Использование метода Kd-деревьев для сегментации областей
            tree->setInputCloud(msg);
            
            // Выделение пригодных областей
            std::vector<pcl::PointIndices> cluster_indices;
            ec.setSearchMethod(tree);
            ec.setInputCloud(msg);
            ec.extract(cluster_indices);
            
            // Кластеризация
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));
            for (auto i = cluster_indices.begin(); i != cluster_indices.end(); i++)
            {
                pcl::PointIndices::Ptr ptr_i (new pcl::PointIndices);
                ptr_i->header = i->header;
                for (auto d: i->indices)
                {
                    ptr_i->indices.push_back(d);
                }
                
                // Выделение кластеров в отдельные облака
                pcl::PointIndices new_i = indexes(ptr_i, ptr_input, msg);
                pcl::PointIndices::Ptr ptr_new_i (new pcl::PointIndices(new_i));
                
                pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
                cloud_cluster = inliers_points(ptr_new_i, ptr_input, cloud_cluster);
                
                // Поиск точки посадки
                pc_landing::LandingPoint srv;
                
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(cloud_cluster, cloud_msg);
                
                srv.request.input = cloud_msg;
                if (client_.call(srv))
                {
                    ROS_INFO("Searching landing point...");
                    pc_landing_area.x = srv.response.x;
                    pc_landing_area.y = srv.response.y;
                    pc_landing_area.z = srv.response.z;
                    pc_landing_area.R = srv.response.R;
                }
                else
                {
                    ROS_ERROR("Service server don't work!");
                }
                
                // Проверка точки на соответствие требованию площади
                pc_radius_m = pc_landing_area.R * pc_range;
                if (M_PI * pow(pc_radius_m, 2) >= pc_square_min)
                {
                    v_lp_mass.push_back(pc_landing_area);
                }
            }
            
            // Извлечение ровной поверхности
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(msg);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*msg);
        }
        
        // Проверка и сохранение наибольшей области посадки
        if (!v_lp_mass.empty())
        {
            float temp = 0;
            for (auto p: v_lp_mass)
            {
                if (p.R > temp)
                {
                    pc_landing_area = p;
                    temp = p.R;
                }
            }
        }
        
        // Вывод
        pc_landing::LandingCoordinates fin_lp;
        fin_lp.x = pc_landing_area.x * pc_range;
        fin_lp.y = pc_landing_area.y * pc_range;
        pub_.publish(fin_lp);
    }
    
    void callback_dist(const std_msgs::Float32Ptr& input)
    {
        pc_range_sensor = input->data;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_detection_slz");
    
    PC_Search space;
    
    ros::Rate loop_rate(0.1);
    while (ros::ok())
    {
        PC_Search space;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
