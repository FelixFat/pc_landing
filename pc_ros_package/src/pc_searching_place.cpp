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
        sub_lp_     = n_.subscribe("/camera/depth_registered/points", 10, &PC_Search::PC_FUNC_LANDING, this);
        client_     = n_.serviceClient<pc_landing::LandingPoint>("landing_point");
    }
    
    // Функция приведения входящих в область облака точек
    pcl::PointCloud<pcl::PointXYZ> PC_CONV_INLIERS(
        pcl::PointIndices::Ptr& inliers,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        pcl::PointCloud<pcl::PointXYZ> output)
    {
        output.width = input->width;
        output.height = input->height;
        output.is_dense = false;
        output.points.resize(output.width * output.height);
        
        for (auto i: inliers->indices)
        {
            output.points[i].x = input->points[i].x;
            output.points[i].y = input->points[i].y;
            output.points[i].z = input->points[i].z;
        }
        
        return(output);
    }

    // Функция приведения индексов облака точек
    pcl::PointIndices PC_CONV_INDIXES(
        pcl::PointIndices::Ptr& inliers,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& msg)
    {
        pcl::PointIndices ind;
        ind.header = inliers->header;
        
        int count = 0;
        for (int i = 0; i < input->size(); i++) {
            if (input->points[i].x == msg->points[inliers->indices[count]].x and
                input->points[i].y == msg->points[inliers->indices[count]].y and
                input->points[i].z == msg->points[inliers->indices[count]].z
                )
            {
                ind.indices.push_back(i);
                count++;
            }
        }
        
        return(ind);
    }

    // Основная функция
    void PC_FUNC_LANDING(
        pcl::PointCloud<pcl::PointXYZ> input_cloud)
    {
        // Расчет расстояния (приведение коэффициентов облака точек в метры)
        float pc_range = pc_range_sensor / input_cloud.at(input_cloud.height/2, input_cloud.width/2).z;
        // Расчет среднего интервала между точками облака
        float pc_points_interval =
            input_cloud.at(input_cloud.height/2, input_cloud.width/2 + 1).x - input_cloud.at(input_cloud.height/2, input_cloud.width/2).x;
        // Расчет минимального числа точек для анализа облаков
        int pc_points_num_min = int(pc_square_min / pc_points_interval);
        
        // Входное облако точек
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_input (new pcl::PointCloud<pcl::PointXYZ>(input_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
        for (auto p: ptr_input->points)
        {
            if (p.x == 0 or p.y == 0 or p.z == 0)
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
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(pc_points_num_min);
        ec.setMaxClusterSize(msg->size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(msg);
        
        while (inliers->indices.size() > pc_points_num_min)
        {
            // Детектирование плоскости методом RANSAC
            plane.setInputCloud(msg);
            plane.segment(*inliers, *coefficients);
            
            // Выделение детектированных областей в отдельное облако
            pcl::PointIndices ind = PC_CONV_INDIXES(inliers, ptr_input, msg);
            pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices(ind));
            
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud = PC_CONV_INLIERS(new_inliers, ptr_input, cloud);
            
            // Использование метода Kd-деревьев для сегментации областей
            tree->setInputCloud(msg);
            
            // Выделение пригодных областей
            ec.extract(cluster_indices);
            
            // Кластеризация
            for (auto i = cluster_indices.begin(); i != cluster_indices.end(); i++)
            {
                pcl::PointIndices::Ptr ptr_i (new pcl::PointIndices);
                ptr_i->header = i->header;
                for (auto d: i->indices)
                {
                    ptr_i->indices.push_back(d);
                }
                
                // Выделение кластеров в отдельные облака
                pcl::PointIndices new_i = PC_CONV_INDIXES(ptr_i, ptr_input, msg);
                pcl::PointIndices::Ptr ptr_new_i (new pcl::PointIndices(new_i));
                
                pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
                cloud_cluster = PC_CONV_INLIERS(ptr_new_i, ptr_input, cloud_cluster);
                
                // Поиск точки посадки
                pc_landing::LandingPoint srv;
                pcl::PointCloud<pcl::PointXYZ>::Ptr srv_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>(cloud_cluster));
                sensor_msgs::PointCloud2 cloud_msg;
                for (int j = 0; j < 100; j++)
                {
                    // Вызов сервиса для поиска точки посадки
                    pcl::toROSMsg(*srv_cloud_cluster, cloud_msg);
                    srv.request.input = cloud_msg;
                    if (client_.call(srv) and srv.response.R > pc_landing_area.R)
                    {
                        pc_landing_area.x = srv.response.x;
                        pc_landing_area.y = srv.response.y;
                        pc_landing_area.z = srv.response.z;
                        pc_landing_area.R = srv.response.R;
                    }
                }
                
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
            float temp = 0.0;
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
    
    return 0;
}
