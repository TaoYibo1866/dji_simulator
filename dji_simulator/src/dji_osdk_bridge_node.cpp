#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dji_osdk_bridge");
    ros::spin();
    return 0;
}