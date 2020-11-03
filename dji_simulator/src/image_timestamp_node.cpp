#include <ros/ros.h>
#include <image_transport/image_transport.h>

image_transport::Publisher img_pub;

void imgCallback(sensor_msgs::Image img_msg)
{
    img_msg.header.stamp = ros::Time::now();
    // ROS_INFO("%ld", img_msg.header.stamp.toNSec());
    img_pub.publish(img_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_timestamp");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    img_pub = it.advertise("out", 1);
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("in", 1, &imgCallback);
    ros::spin();
    return 0;
}