#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <pc_landing/Landing.h>
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
        pub_        = n_.advertise<pc_landing::Landing>("/copter/point_landing", 1);
        sub_dist_   = n_.subscribe("/rangeginder/range", 10, &PC_Search::callback_dist, this);
        sub_lp_     = n_.subscribe("/camera/depth_registered/points", 10, &PC_Search::callback_lp, this);
        client_     = n_.serviceClient<pc_landing::LandingPoint>("landing_point");
    }
    
    t_landing_circle landing_point(pcl::PointCloud<pcl::PointXYZ> cloud)
    {
        int gw, gh;
        while (true)
        {
            gw = rand() % cloud.width;
            gh = rand() % cloud.height;
            
            if (cloud.at(gw, gh).x == 0 and
                cloud.at(gw, gh).y == 0 and
                cloud.at(gw, gh).z == 0)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        
        bool state = true;
        bool flag = false;
        int R = 1;
        
        std::vector<t_frame> po;
        
        float x, y, z;
        
        int goal_w, goal_h, goal_R = 0;
        
        while (true)
        {        
            for (int h = gh - R; h <= gh + R; h++)
            {
                for (int w = gw - R; w <= gw + R; w++)
                {
                    if (pow(w - gw, 2) + pow(h - gh, 2) > pow(R, 2))
                    {
                        continue;
                    }
                    
                    if (w < 0 or w > cloud.width-1 or
                        h < 0 or h > cloud.height-1)
                    {
                        po.push_back({ gw, gh });
                        
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
                        po.push_back({ gw, gh });
                        
                        gw -= round((w - gw)/R);
                        gh -= round((h - gh)/R);
                        
                        state = false;
                        break;
                    }
                }
                
                if (!state)
                {
                    break;
                }
            }
            
            if (state)
            {
                goal_w = gw;
                goal_h = gh;
                goal_R = R;
                
                R++;
            }
            else
            {
                state = true;
                for (auto p: po)
                {
                    if (gw == p.width and gh == p.height)
                    {
                        flag = true;
                        break;
                    }
                }
            }
            
            if (flag)
            {
                break;
            }
        }
        
        t_landing_circle circle = {0.0, 0.0, 0.0, 0.0};
        
        if (goal_R > 0.0)
        {
            float Radius = abs(cloud.at(goal_w, goal_h).x - cloud.at(goal_w - goal_R, goal_h).x);
            circle = {
                cloud.at(goal_w, goal_h).x,
                cloud.at(goal_w, goal_h).y,
                cloud.at(goal_w, goal_h).z,
                Radius
            };
        }
        
        return(circle);
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
        
        pc_landing_area = { 0.0, 0.0, 0.0, 0.0 };
        while (true)
        {
            // Детектирование плоскости методом RANSAC
            plane.setInputCloud(msg);
            plane.segment(*inliers, *coefficients);
            
            if (inliers->indices.size() < pc_points_num_min)
            {
                break;
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
                pc_landing_area = landing_point(cloud_cluster);
                
//                 pc_landing::LandingPoint srv;
//                 pcl::PointCloud<pcl::PointXYZ>::Ptr srv_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>(cloud_cluster));
//                 sensor_msgs::PointCloud2Ptr cloud_msg (new sensor_msgs::PointCloud2);
//                 
//                 // Вызов сервиса для поиска точки посадки
//                 pcl::toROSMsg(*srv_cloud_cluster, *cloud_msg);
//                 
//                 srv.request.input = *cloud_msg;
//                 if (client_.call(srv))
//                 {
//                     pc_landing_area.x = srv.response.x;
//                     pc_landing_area.y = srv.response.y;
//                     pc_landing_area.z = srv.response.z;
//                     pc_landing_area.R = srv.response.R;
//                 }
                
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
            v_lp_mass.clear();
        }
        
        // Вывод
        pc_landing::Landing fin_lp;
        fin_lp.x = pc_landing_area.x;
        fin_lp.y = pc_landing_area.y;
        fin_lp.z = pc_landing_area.z;
        fin_lp.R = pc_landing_area.R;
        pub_.publish(fin_lp);
    }
    
    void callback_dist(const sensor_msgs::RangeConstPtr& input)
    {
        pc_range_sensor = input->range + 0.025;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_searching_place");
    
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
