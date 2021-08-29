#include <ros/ros.h>

class PC_Search
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    
public:
    PC_Search()
    {
        pub_ = n_.advertise<pc_landing::Landing>("/copter/point_landing", 1);
        sub_ = n_.subscribe("/camera/depth_registered/points", 10, &PC_Search::callback, this);
    }
    
    void callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
    {
        return;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_searching_place");
    PC_Search space;
    ros::spinOnce();
    
    return 0;
}
