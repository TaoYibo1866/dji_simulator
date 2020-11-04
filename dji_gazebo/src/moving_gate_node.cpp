#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>

using gazebo_msgs::SetModelState;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moving_gate");
    ros::NodeHandle nh;
    ros::ServiceClient set_model_client = nh.serviceClient<SetModelState>("gazebo/set_model_state");
    SetModelState srv;
    srv.request.model_state.model_name = "square_gate";
    srv.request.model_state.pose.position.x = -3;
    srv.request.model_state.pose.position.y = 15;
    srv.request.model_state.pose.position.z = 0;

    int sign = 1;
    ros::Rate r(50);
    while(ros::ok())
    {
        srv.request.model_state.pose.position.x += sign * 0.5 * 1/50;
        if(sign > 0 && srv.request.model_state.pose.position.x > 3)
            sign = -1;
        if(sign < 0 && srv.request.model_state.pose.position.x < -3)
            sign = 1;
        set_model_client.call(srv);
        r.sleep();
    }
    return 0;
}