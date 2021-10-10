#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <librealsense2/rs.hpp>

class PC_Distance
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    
public:
    PC_Distance()
    {        
        pub_ = n_.advertise<std_msgs::Float32>("/copter/dist_to_center", 1);
        
        std_msgs::Float32 msg;
        msg.data = get_distance();
        
        ROS_INFO("Distance is %f meters away!", (float)msg.data);
        
        pub_.publish(msg);
    }
    
    float get_distance(void)
    {
        rs2::pipeline p;
        p.start();
        
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        
        int width = depth.get_width();
        int height = depth.get_height();
        float dist_to_center = depth.get_distance(width / 2, height / 2);
        
        return dist_to_center;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_distance");
    
    PC_Distance distance;
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        PC_Distance distance;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
