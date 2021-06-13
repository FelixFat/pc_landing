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

#define PI 3.14159265

struct frame
{
    int w;
    int h;
};

struct landing
{
    float x;
    float y;
    float z;
    float R;
};

std::vector<landing> lp;
landing land = {0.0, 0.0, 0.0, 0.0};

landing landing_point(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    int gw, gh;
    while (true)
    {
        gw = rand() % cloud.width;
        gh = rand() % cloud.height;
        if (cloud.at(gw, gh).x == 0 and cloud.at(gw, gh).y == 0 and cloud.at(gw, gh).z == 0)
            continue;
        else if ((gw == 0 and gh == 0) and (gw == cloud.width-1 and gh == 0) and (gw == 0 and gh == cloud.height-1) and (gw == cloud.width-1 and gh == cloud.height-1))
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
    
    landing circle = { 0.0, 0.0, 0.0, 0.0 };
    
    if (goal_R > 0)
    {
        float Radius = abs(cloud.at(goal_w, goal_h).x - cloud.at(goal_w - goal_R, goal_h).x);
    
        circle = { cloud.at(goal_w, goal_h).x, cloud.at(goal_w, goal_h).y, cloud.at(goal_w, goal_h).z, Radius };
    }
    
    return(circle);
}

pcl::PointCloud<pcl::PointXYZ> organize(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    if (cloud.height != 1)
    {
        std::cout << "cloud is organized!" << std::endl;
        return(cloud);
    }
    
    int w, h;
    if (cloud.size() == 424 * 240)
    {
        w = 424;
        h = 240;
    }
    else if (cloud.size() == 480 * 270)
    {
        w = 480;
        h = 270;
    }
    else if (cloud.size() == 640 * 360)
    {
        w = 640;
        h = 360;
    }
    else if (cloud.size() == 640 * 480)
    {
        w = 640;
        h = 480;
    }
    else if (cloud.size() == 848 * 480)
    {
        w = 848;
        h = 480;
    }
    else if (cloud.size() == 1280 * 720)
    {
        w = 1280;
        h = 720;
    }
    
    pcl::PointCloud<pcl::PointXYZ> org;
    org.width = w;
    org.height = h;
    org.is_dense = false;
    org.points.resize(org.width * org.height);
    
    for (int j = 0; j < org.height; j++)
    {
        for (int i = 0; i < org.width; i++)
        {
            org.at(i, j).x = cloud.points[i + w*j].x;
            org.at(i, j).y = cloud.points[i + w*j].y;
            org.at(i, j).z = cloud.points[i + w*j].z;
        }
    }
    
    return (org);
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
    pcl::PointIndices ind;
    ind.header = inliers->header;
    
    int count = 0;
    for (int i = 0; i < input->size(); i++)
    {
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

void viss(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    // Вывод облака точек на экран
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(ptr_cloud);
    
    int user_data;
    while (!viewer.wasStopped())
        user_data++;
}

void callback(pcl::PointCloud<pcl::PointXYZ> input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_input (new pcl::PointCloud<pcl::PointXYZ>(input));
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    for (auto p: ptr_input->points)
    {
        if (p.x == 0 or p.y == 0 or p.z == 0)
            continue;
        else
            msg->push_back(p);
    }
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> plane;
    plane.setOptimizeCoefficients(true);
    plane.setModelType(pcl::SACMODEL_PLANE);
    plane.setMethodType(pcl::SAC_RANSAC);
    plane.setMaxIterations(1000);
    plane.setDistanceThreshold(0.01);
    
    while (true)
    {
        std::cout << "RANSAC..." << std::endl;
        plane.setInputCloud(msg);
        plane.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < 0.3 * ptr_input->size())
        {
            PCL_ERROR("Could not estimate a PLANAR model for the given dataset.\n");
            break;
        }
        
        pcl::PointIndices ind = indexes(inliers, ptr_input, msg);
        pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices(ind));
        
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud = inliers_points(new_inliers, ptr_input, cloud);
        
        std::cout << "\tKd-Tree..." << std::endl;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(msg);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(0.01 * msg->size());
        ec.setMaxClusterSize(msg->size());
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
            
            std::cout << "\tclusterung..." << std::endl;
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
                
                viss(cloud_cluster);
                
                land = landing_point(cloud_cluster);
                
                if (PI * pow(land.R, 2) >= PI * pow(0.002, 2))
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
    }
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 640;
    cloud.height = 480;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    std::cout << "cloud size: " << cloud.size() << std::endl;
    
//     std::cout << "organizing..." << std::endl;
//     cloud = organize(cloud);
    
    std::cout << "creating..." << std::endl;
    for (int i = 0; i < 80; i++)
    {
        for (int j = 0; j < 400; j++)
        {
            cloud.at(j, i).x = float(j)/1000;
            cloud.at(j, i).y = float(i)/1000;
            cloud.at(j, i).z = float(0.2);
        }
    }
    
    for (int i = 400; i < 480; i++)
    {
        for (int j = 0; j < 400; j++)
        {
            cloud.at(j, i).x = float(j)/1000;
            cloud.at(j, i).y = float(i)/1000;
            cloud.at(j, i).z = float(0.2);
        }
    }
    
    for (int i = 0; i < 480; i++)
    {
        for (int j = 400; j < 640; j++)
        {
            cloud.at(j, i).x = float(j)/1000;
            cloud.at(j, i).y = float(i)/1000;
            cloud.at(j, i).z = float(0.2);
        }
    }
    
    for (int i = 120; i < 360; i++)
    {
        for (int j = 0; j < 240; j++)
        {
            cloud.at(j, i).x = float(j)/1000;
            cloud.at(j, i).y = float(i)/1000;
            cloud.at(j, i).z = float(0.2);
        }
    }

    std::cout << "searching landing point..." << std::endl;
    callback(cloud);
    std::cout << "\tx: " << land.x << ", y: " << land.y << ", z: " << land.z << ", R: " << land.R << std::endl;
    
    std::cout << "end " << std::endl;
    return 0;
}
