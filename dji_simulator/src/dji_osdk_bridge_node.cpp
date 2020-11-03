#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <gazebo_msgs/ModelState.h>

using geometry_msgs::PointStamped;
using geometry_msgs::QuaternionStamped;
using geometry_msgs::PointStampedConstPtr;
using geometry_msgs::QuaternionStampedConstPtr;
using message_filters::sync_policies::ApproximateTime;
using gazebo_msgs::ModelState;
typedef ApproximateTime<PointStamped, QuaternionStamped> SyncPolicy;

int freq_ctrl=1, cnt=0;
double x_offset, y_offset, z_offset, yaw_offset;
std::string model_name;
ros::Publisher model_state_pub;

void stateCallback(const PointStampedConstPtr& local_position, const QuaternionStampedConstPtr& attitude)
{
    if(++cnt < freq_ctrl)
        return;
    cnt = 0;

    tf2::Vector3 origin;
    tf2::Quaternion rotation;
    tf2::fromMsg(local_position->point, origin);
    tf2::fromMsg(attitude->quaternion, rotation);
    tf2::Transform iTw, iTb, wTb;
    iTw.setOrigin(tf2::Vector3(x_offset, y_offset, z_offset));
    iTw.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_offset));
    iTb.setOrigin(origin);
    iTb.setRotation(rotation);
    wTb = iTw.inverse() * iTb;

    ModelState state_msg;
    state_msg.model_name = model_name;
    state_msg.reference_frame = "world";
    tf2::toMsg(wTb, state_msg.pose);
    model_state_pub.publish(state_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dji_osdk_bridge");
    ros::NodeHandle nh;
    ros::param::param<std::string>("~model_name", model_name, "dji_m100");
    ros::param::param<int>("~div", freq_ctrl, 1);
    ros::param::param<double>("~x_offset", x_offset, 0);
    ros::param::param<double>("~y_offset", y_offset, 0);
    ros::param::param<double>("~z_offset", z_offset, 0);
    ros::param::param<double>("~yaw_offset", yaw_offset, 0);
    model_state_pub = nh.advertise<ModelState>("gazebo/set_model_state", 1);
    message_filters::Subscriber<PointStamped> local_position_sub(nh, "dji_sdk/local_position", 1);
    message_filters::Subscriber<QuaternionStamped> attitude_sub(nh, "dji_sdk/attitude", 1);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), local_position_sub, attitude_sub);
    sync.registerCallback(boost::bind(&stateCallback, _1,_2));
    ros::spin();
    return 0;
}