#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2Ptr input_cloud (new sensor_msgs::PointCloud2(req.input));
    pcl::fromROSMsg(*input_cloud, *cloud);
    
    int gw, gh;
    while (true)
    {
        gw = rand() % cloud->width;
        gh = rand() % cloud->height;
        
        if (cloud->at(gw, gh).x == 0 and
            cloud->at(gw, gh).y == 0 and
            cloud->at(gw, gh).z == 0)
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
                
                if (w < 0 or w > cloud->width-1 or
                    h < 0 or h > cloud->height-1)
                {
                    po.push_back({ gw, gh });
                    
                    gw -= round((w - gw)/R);
                    gh -= round((h - gh)/R);
                    
                    state = false;
                    break;
                }
                
                x = cloud->at(w, h).x;
                y = cloud->at(w, h).y;
                z = cloud->at(w, h).z;
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
    
    t_landing_circle circle = {1.0, 2.0, 3.0, 4.0};
    
    if (goal_R > 0.0)
    {
        float Radius = abs(cloud->at(goal_w, goal_h).x - cloud->at(goal_w - goal_R, goal_h).x);
        circle = {
            cloud->at(goal_w, goal_h).x,
            cloud->at(goal_w, goal_h).y,
            cloud->at(goal_w, goal_h).z,
            Radius
        };
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
