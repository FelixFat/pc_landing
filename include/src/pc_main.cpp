#include <iostream>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "pc_landing.cpp"

void viss(pcl::PointCloud<pcl::PointXYZ> cloud);

// Главная функция
int main(int argc, char** argv)
{
    // Инициализация облака точек
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 640;
    cloud.height = 480;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    std::cout << "cloud size: " << cloud.size() << std::endl;
    
    // Формирование облака точек тестового набора
    std::cout << "creating..." << std::endl;
    for (int i = 0; i < 80; i++) {
        for (int j = 0; j < 400; j++) {
            cloud.at(j, i).x = float(j)/10000.0f;
            cloud.at(j, i).y = float(i)/10000.0f;
            cloud.at(j, i).z = 0.2f;
        }
    }
    
    for (int i = 400; i < 480; i++) {
        for (int j = 0; j < 400; j++) {
            cloud.at(j, i).x = float(j)/10000.0f;
            cloud.at(j, i).y = float(i)/10000.0f;
            cloud.at(j, i).z = 0.2f;
        }
    }
    
    for (int i = 0; i < 480; i++) {
        for (int j = 400; j < 640; j++) {
            cloud.at(j, i).x = float(j)/10000.0f;
            cloud.at(j, i).y = float(i)/10000.0f;
            cloud.at(j, i).z = 0.2f;
        }
    }
    
    for (int i = 120; i < 360; i++) {
        for (int j = 0; j < 240; j++) {
            cloud.at(j, i).x = float(j)/10000.0f;
            cloud.at(j, i).y = float(i)/10000.0f;
            cloud.at(j, i).z = 0.2f;
        }
    }

    viss(cloud);
    
    //Вывод результата
    std::cout << "searching landing point..." << std::endl;
    PC_FUNC_LANDING(cloud);
    std::cout <<
        "\tx: " << pc_landing_area.x <<
        ", y: " << pc_landing_area.y <<
        ", z: " << pc_landing_area.z <<
        ", R: " << pc_landing_area.R << std::endl;
    
    std::cout << "end " << std::endl;
    return 0;
}

// Функция вывода облака точек на экран
void viss(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(ptr_cloud);
    
    int user_data;
    while (!viewer.wasStopped()) {
        user_data++;
    }
}
