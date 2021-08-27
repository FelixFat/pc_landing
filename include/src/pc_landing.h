#ifndef __PC_LANDING_H__
#define __PC_LANDING_H__

#include <pcl/point_types.h>

#define PI 3.14159265

// t_###    - структура
// pc_###   - переменная
// v_###    - вектор
// 
// 

// Структура кадра
struct t_frame {
    int width;
    int height;
};

// Структура посадочной окружности для метода расширения
struct t_landing_circle {
    float x;
    float y;
    float z;
    float R;
};

// Максимально допустимый угол наклона плоскости
double pc_model_angle = 20.0;

// Минимально допустимая площадь области посадки в [м^2]
float pc_square_min = 0.126;

// Вектор, хранящий предыдущие окружности
std::vector<t_landing_circle> v_lp_mass;

// Найденая область посадки
t_landing_circle pc_landing_area = { 0.0, 0.0, 0.0, 0.0 };

/****************
    Функции
****************/
// landing_point
t_landing_circle PC_SEARCH_POINT(
    pcl::PointCloud<pcl::PointXYZ> cloud);

// inliers_points
pcl::PointCloud<pcl::PointXYZ> PC_CONV_INLIERS(
    pcl::PointIndices::Ptr& inliers,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ> output);

// indexes
pcl::PointIndices PC_CONV_INDIXES(
    pcl::PointIndices::Ptr& inliers,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);

// callback
void PC_FUNC_LANDING(
    pcl::PointCloud<pcl::PointXYZ> input_cloud);

#endif  // __PC_LANDING_H__
