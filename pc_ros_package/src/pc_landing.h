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

// Расстояние до земли с дальномера в [м]
float pc_range_sensor = 1.0;

// Приведенный радиус из облака точек в метры
float pc_radius_m = 0.0;

// Вектор, хранящий предыдущие окружности
std::vector<t_landing_circle> v_lp_mass;

// Найденая область посадки
t_landing_circle pc_landing_area = { 0.0, 0.0, 0.0, 0.0 };

#endif  // __PC_LANDING_H__
