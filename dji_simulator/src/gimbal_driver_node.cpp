#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/Joy.h>

using gazebo_msgs::SetModelConfiguration;
using gazebo_msgs::GetJointProperties;
using sensor_msgs::Joy;
using sensor_msgs::JoyConstPtr;

ros::Publisher joint_pub;
ros::ServiceClient set_joint_client;
ros::ServiceClient get_joint_client;

SetModelConfiguration set_srv;
GetJointProperties get_srv0, get_srv1, get_srv2;

void cmdCallback(const JoyConstPtr& cmd)
{
    ROS_ASSERT(cmd->axes.size() == 3);
    set_srv.request.joint_positions.clear();
    set_srv.request.joint_positions.push_back(cmd->axes[0]);
    set_srv.request.joint_positions.push_back(cmd->axes[1]);
    set_srv.request.joint_positions.push_back(cmd->axes[2]);
    set_joint_client.call(set_srv);
}

void timerCallback(const ros::TimerEvent& event)
{
    Joy joint_state;
    joint_state.header.stamp = ros::Time::now();
    if(get_joint_client.call(get_srv0)
    && get_joint_client.call(get_srv1)
    && get_joint_client.call(get_srv2)
    && get_srv0.response.success
    && get_srv1.response.success
    && get_srv2.response.success)
    {
        joint_state.axes.push_back(get_srv0.response.position[0]);
        joint_state.axes.push_back(get_srv1.response.position[0]);
        joint_state.axes.push_back(get_srv2.response.position[0]);
        joint_pub.publish(joint_state);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_driver");
    ros::NodeHandle nh;
    set_joint_client = nh.serviceClient<SetModelConfiguration>("gazebo/set_model_configuration");
    get_joint_client = nh.serviceClient<GetJointProperties>("gazebo/get_joint_properties");
    set_joint_client.waitForExistence();
    get_joint_client.waitForExistence();
    ros::param::param<std::string>("~model_name", set_srv.request.model_name, "dji_m100");
    ros::param::param<std::string>("~urdf_param_name", set_srv.request.urdf_param_name, "robot_description");
    std::vector<std::string> joint_names(3);
    ros::param::param<std::string>("~joint0", joint_names[0], "gimbal_camera/gimbal_joint_0");
    ros::param::param<std::string>("~joint1", joint_names[1], "gimbal_camera/gimbal_joint_1");
    ros::param::param<std::string>("~joint2", joint_names[2], "gimbal_camera/gimbal_joint_2");
    set_srv.request.joint_names = joint_names;
    get_srv0.request.joint_name = joint_names[0];
    get_srv1.request.joint_name = joint_names[1];
    get_srv2.request.joint_name = joint_names[2];
    ros::Subscriber cmd_sub = nh.subscribe<Joy>("gimbal_angle_cmd", 1, cmdCallback);
    joint_pub = nh.advertise<Joy>("gimbal_angle", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(1/50), timerCallback, false, false);
    ros::Duration(5).sleep(); // wait for model, otherwise gazebo gui stuck on startup
    timer.start();
    ros::spin();
    return 0;
}