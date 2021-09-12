#include <ros/ros.h>
#include <pc_landing/LandingPoint.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

bool landing_point(pc_landing::LandingPoint::Request  &req,
                   pc_landing::LandingPoint::Response &res)
{
    // Инициализация входного облака точек в формат PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(req.input, cloud);
    
    // Выбор случайной точки в облаке
    int gw, gh;
    while (true) {
        gw = rand() % cloud.width;
        gh = rand() % cloud.height;
        
        if (cloud.at(gw, gh).x == 0 and
            cloud.at(gw, gh).y == 0 and
            cloud.at(gw, gh).z == 0)
        {
            continue;
        }
        else if ((gw == 0 and gh == 0) and
                 (gw == cloud.width - 1 and gh == 0) and
                 (gw == 0 and gh == cloud.height - 1) and
                 (gw == cloud.width - 1 and gh == cloud.height - 1))
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
    int step = 1;
    
    std::vector<t_frame> po;
    
    float x, y, z;
    int goal_w, goal_h, goal_R = 0;
    
    // Поиск точки посадки
    while (true)
    {
        for (int h = gh - R; h <= gh + R; h += step)
        {
            for (int w = gw - R; w <= gw + R; w += step)
            {
                if (pow(w - gw, 2) + pow(h - gh, 2) > pow(R, 2))
                {
                    continue;
                }
                
                x = cloud.at(w, h).x;
                y = cloud.at(w, h).y;
                z = cloud.at(w, h).z;
                if (w < 0 or w > cloud.width - 1 or
                    h < 0 or h > cloud.height - 1 or
                   (x == 0 and y == 0 and z == 0))
                {
                    t_frame p = { gw, gh };
                    po.push_back(p);
                    
                    gw -= round((w - gw) / R);
                    gh -= round((h - gh) / R);
                    
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
            
            R += step;
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
    
    // Анализ результата и настройка вывода
    t_landing_circle circle = { 0.0, 0.0, 0.0, 0.0 };
    if (goal_R > 0.0)
    {
        float Radius = fabs(cloud.at(goal_w, goal_h).x - cloud.at(goal_w - goal_R, goal_h).x);
        circle = { cloud.at(goal_w, goal_h).x, cloud.at(goal_w, goal_h).y, cloud.at(goal_w, goal_h).z, Radius };
    }
    
    res.x = circle.x;
    res.y = circle.y;
    res.z = circle.z;
    res.R = circle.R;
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_point");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/copter/landing_point", landing_point);
    
    ros::spin();

    return 0;
}
