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

#define PI 3.14159265

struct landing
{
    float x;
    float y;
    float z;
    float R;
};

struct frame
{
    int w;
    int h;
};

class PC_Search
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_lp_;
    ros::Subscriber sub_dist_;
    
    float dist_ = 1;
    
public:
    PC_Search()
    {
        pub_ = n_.advertise<pc_landing::Landing>("/copter/point_landing", 1);
        sub_lp_ = n_.subscribe("/camera/depth_registered/points", 10, &PC_Search::callback_lp, this);
        sub_dist_ = n_.subscribe("/rangeginder/range", 10, &PC_Search::callback_dist, this);
    }
    
    landing landing_point(pcl::PointCloud<pcl::PointXYZ> cloud)
    {
        int gw, gh;
        while (true)
        {
            gw = rand() % cloud.width;
            gh = rand() % cloud.height;
            if (cloud.at(gw, gh).x == 0 and cloud.at(gw, gh).y == 0 and cloud.at(gw, gh).z == 0)
                continue;
            else
                break;
        }
        
        bool state = true;
        bool flag = false;
        int R = 1;
        int step = 1;
        
        std::vector<frame> po;
        
        float x, y, z;
        
        int goal_w, goal_h, goal_R = 0;
        
        while (true)
        {        
            for (int h = gh - R; h <= gh + R; h += step)
            {
                for (int w = gw - R; w <= gw + R; w += step)
                {
                    if (pow(w - gw, 2) + pow(h - gh, 2) > pow(R, 2))
                        continue;
                    
                    if (w < 0 or w > cloud.width-1 or h < 0 or h > cloud.height-1)
                    {
                        frame p = { gw, gh };
                        po.push_back(p);
                        
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
                        frame p = { gw, gh };
                        po.push_back(p);
                        
                        gw -= round((w - gw)/R);
                        gh -= round((h - gh)/R);
                        
                        state = false;
                        break;
                    }
                }
                if (!state)
                    break;
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
                    if (gw == p.w and gh == p.h)
                    {
                        flag = true;
                        break;
                    }
                }
            }
            
            if (flag)
                break;
        }
        
        landing circle = {0.0, 0.0, 0.0, 0.0};
        
        if (goal_R > 0)
        {
            float Radius = abs(cloud.at(goal_w, goal_h).x - cloud.at(goal_w - goal_R, goal_h).x);
        
            circle = { cloud.at(goal_w, goal_h).x, cloud.at(goal_w, goal_h).y, cloud.at(goal_w, goal_h).z, Radius };
        }
        
        return(circle);
    }

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

    void callback_lp(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_input (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *ptr_input);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
        for (auto p: ptr_input->points)
        {
            if (isnan(p.x) or isnan(p.y) or isnan(p.z))
                continue;
            else
                msg->push_back(p);
        }
        
        float k = dist_/(ptr_input->points[ptr_input->size()/2].z);
        
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> plane;
        plane.setOptimizeCoefficients(true);
        plane.setModelType(pcl::SACMODEL_PLANE);
        plane.setMethodType(pcl::SAC_RANSAC);
        plane.setMaxIterations(1000);
        plane.setDistanceThreshold(0.01);
        
        std::vector<landing> lp;
        landing land = { 0.0, 0.0, 0.0, 0.0 };
        while (true)
        {
            plane.setInputCloud(msg);
            plane.segment(*inliers, *coefficients);
            
            if (inliers->indices.size() < int(0.4/k))
                break;
            
            pcl::PointIndices ind = indexes(inliers, ptr_input, msg);
            pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices(ind));
            
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud = inliers_points(new_inliers, ptr_input, cloud);
            
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(msg);
            
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.01);
            ec.setMinClusterSize(0.4/k);
            ec.setSearchMethod(tree);
            ec.setInputCloud(msg);
            ec.extract(cluster_indices);
            
            float normal[3] = { 0, 0, 1 };
            float normal_place[3] = { coefficients->values[0], coefficients->values[1], coefficients->values[2] };
            float dot = normal[0]*normal_place[0] + normal[1]*normal_place[1] + normal[2]*normal_place[2];
            float len_normal = sqrt(pow(normal[0], 2) + pow(normal[1], 2) + pow(normal[2], 2));
            float len_normal_place = sqrt(pow(normal_place[0], 2) + pow(normal_place[1], 2) + pow(normal_place[2], 2));
            float angle = abs(acos(dot/(len_normal*len_normal_place)) * 180.0/PI);
            
            if (angle <= 20.0)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));
                
                for (auto i = cluster_indices.begin(); i != cluster_indices.end(); i++)
                {
                    pcl::PointIndices::Ptr ptr_i (new pcl::PointIndices);
                    ptr_i->header = i->header;
                    for (auto d: i->indices)
                        ptr_i->indices.push_back(d);
                    
                    pcl::PointIndices new_i = indexes(ptr_i, ptr_input, msg);
                    pcl::PointIndices::Ptr ptr_new_i (new pcl::PointIndices(new_i));
                    
                    pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
                    cloud_cluster = inliers_points(ptr_new_i, ptr_input, cloud_cluster);
                    
                    land = landing_point(cloud_cluster);
                    
                    if (PI * pow(land.R * k, 2) >= PI * pow(0.2, 2))
                        lp.push_back(land);
                }
            }
            
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(msg);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*msg);
        }
        
        if (!lp.empty())
        {
            float temp = 0;
            for (auto p: lp)
                if (p.R > temp)
                {
                    land = p;
                    temp = p.R;
                }
            lp.clear();
        }
        
        pc_landing::Landing fin_lp;
        fin_lp.x = land.x * k;
        fin_lp.y = land.y * k;
        fin_lp.z = land.z * k;
        fin_lp.R = land.R * k;
        pub_.publish(fin_lp);
    }
    
    void callback_dist(const sensor_msgs::RangeConstPtr& input)
    {
        dist_ = input->range;
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
