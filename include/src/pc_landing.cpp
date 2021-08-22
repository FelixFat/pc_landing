#include <iostream>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/visualization/cloud_viewer.h>

// Функция поиска точки приземления
t_landing_circle PC_SEARCH_POINT(
    pcl::PointCloud<pcl::PointXYZ> cloud)
{
    // Случайный выбор точки для начала поиска точки посадки
    int gw, gh;
    while (true) {
        gw = rand() % cloud.width;
        gh = rand() % cloud.height;
        if (cloud.at(gw, gh).x == 0 and cloud.at(gw, gh).y == 0 and cloud.at(gw, gh).z == 0) {
            continue;
        }
        else if ((gw == 0 and gh == 0) and (gw == cloud.width-1 and gh == 0) and (gw == 0 and gh == cloud.height-1) and (gw == cloud.width-1 and gh == cloud.height-1)) {
            continue;
        }
        else {
            break;
        }
    }
    
    bool state = true;
    bool flag = false;
    int R = 1;
    int step = 1;
    
    std::vector<t_frame> po;
    
    float x, y, z;
    
    int goal_w, goal_h, goal_R = 0;
    
    // Поиск точки посадки
    while (true) {
        for (int h = gh - R; h <= gh + R; h += step) {
            for (int w = gw - R; w <= gw + R; w += step) {
                if (pow(w - gw, 2) + pow(h - gh, 2) > pow(R, 2)) {
                    continue;
                }
                
                if (w < 0 or w > cloud.width-1 or h < 0 or h > cloud.height-1) {
                    t_frame p = { gw, gh };
                    po.push_back(p);
                    
                    gw -= round((w - gw)/R);
                    gh -= round((h - gh)/R);
                    
                    state = false;
                    break;
                }
                
                x = cloud.at(w, h).x;
                y = cloud.at(w, h).y;
                z = cloud.at(w, h).z;
                if (x == 0 and y == 0 and z == 0) {
                    t_frame p = { gw, gh };
                    po.push_back(p);
                    
                    gw -= round((w - gw)/R);
                    gh -= round((h - gh)/R);
                    
                    state = false;
                    break;
                }
            }
            if (!state) {
                break;
            }
        }
        
        if (state) {
            goal_w = gw;
            goal_h = gh;
            goal_R = R;
            
            R += step;
        }
        else {
            state = true;
            for (auto p: po) {
                if (gw == p.w and gh == p.h) {
                    flag = true;
                    break;
                }
            }
        }
        
        if (flag) {
            break;
        }
    }
    
    // Анализ результата и настройка вывода
    t_landing_circle circle = { 0.0, 0.0, 0.0, 0.0 };
    
    if (goal_R > 0) {
        float Radius = fabs(cloud.at(goal_w, goal_h).x - cloud.at(goal_w - goal_R, goal_h).x);
        circle = { cloud.at(goal_w, goal_h).x, cloud.at(goal_w, goal_h).y, cloud.at(goal_w, goal_h).z, Radius };
    }
    
    return(circle);
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
    
    for (auto i: inliers->indices) {
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_input (new pcl::PointCloud<pcl::PointXYZ>(input_cloud));
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    for (auto p: ptr_input->points) {
        if (p.x == 0 or p.y == 0 or p.z == 0) {
            continue;
        }
        else {
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
    plane.setMaxIterations(1000);
    plane.setDistanceThreshold(0.01);
    
    // Настройка модели сегментации
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    // Настройка модели выделение пригодных областей
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01);
    //
    //  КОЭФФИЦИЕНТ
    //
    ec.setMinClusterSize(0.01 * msg->size());
    ec.setMaxClusterSize(msg->size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(msg);
    
    while (true) {
        // Детектирование плоскости методом RANSAC
        std::cout << "RANSAC..." << std::endl;
        plane.setInputCloud(msg);
        plane.segment(*inliers, *coefficients);
        
        //
        //  КОЭФФИЦИЕНТ
        //
        if (inliers->indices.size() < 0.3 * ptr_input->size()) {
            PCL_ERROR("Could not estimate a PLANAR model for the given dataset.\n");
            break;
        }
        
        // Выделение детектированных областей в отдельное облако
        pcl::PointIndices ind = indexes(inliers, ptr_input, msg);
        pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices(ind));
        
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud = inliers_points(new_inliers, ptr_input, cloud);
        
        // Использование метода Kd-деревьев для сегментации областей
        std::cout << "\tKd-Tree..." << std::endl;
        tree->setInputCloud(msg);
        
        // Выделение пригодных областей
        ec.extract(cluster_indices);
        
        //
        //  УБРАТЬ РАСЧЕТ УГЛА И ВЫНЕСТИ В RANSAC
        //
        //  Определение угла наклоны найденной области
        float normal[3] = { 0, 0, 1 };
        float normal_place[3] = { coefficients->values[0], coefficients->values[1], coefficients->values[2] };
        float dot = normal[0]*normal_place[0] + normal[1]*normal_place[1] + normal[2]*normal_place[2];
        float len_normal = sqrt(pow(normal[0], 2) + pow(normal[1], 2) + pow(normal[2], 2));
        float len_normal_place = sqrt(pow(normal_place[0], 2) + pow(normal_place[1], 2) + pow(normal_place[2], 2));
        float angle = abs(acos(dot/(len_normal*len_normal_place)) * 180.0/PI);
        
        if (angle <= 20.0f) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));
            
            // Кластеризация
            std::cout << "\tclusterung..." << std::endl;
            for (auto i = cluster_indices.begin(); i != cluster_indices.end(); i++) {
                pcl::PointIndices::Ptr ptr_i (new pcl::PointIndices);
                ptr_i->header = i->header;
                for (auto d: i->indices) {
                    ptr_i->indices.push_back(d);
                }
                
                // Выделение кластеров в отдельные облака
                pcl::PointIndices new_i = indexes(ptr_i, ptr_input, msg);
                pcl::PointIndices::Ptr ptr_new_i (new pcl::PointIndices(new_i));
                
                pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
                cloud_cluster = inliers_points(ptr_new_i, ptr_input, cloud_cluster);
                
                // Поиск точки посадки
                v_lp_mass = landing_point(cloud_cluster);
                
                if (PI * pow(v_lp_mass.R, 2) >= PI * pow(0.002f, 2)) {
                    v_lp_mass.push_back(v_lp_mass);
                }
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
    if (!v_lp_mass.empty()) {
        float temp = 0;
        for (auto p: v_lp_mass) {
            if (p.R > temp) {
                v_lp_mass = p;
                temp = p.R;
            }
        }
    }
}
