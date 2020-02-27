#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include <sstream>
#include "/home/viki/catkin_ws/src/nmpc_robo/include/ToOffboard.h" // Trocar aqui para seu computador tambem

using namespace std;

geometry_msgs::TwistStamped thisMsg;

void chatterCallbackToOffboard(const geometry_msgs::TwistStamped& msg){
    thisMsg = msg;//*msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Cpp2Offb");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("/setpoint_offboard_TwstS", 1, chatterCallbackToOffboard);
		

    ros::Publisher chatter_pub = n.advertise<vant::ToOffboard>("/setpoint_offboard",1);
    vant::ToOffboard pubMsg;

    while(ros::ok())
    {
        ros::spinOnce();
        pubMsg.index = 2;
        pubMsg.TwistStamped.twist.linear.x = thisMsg.twist.linear.x;
        pubMsg.TwistStamped.twist.linear.y = thisMsg.twist.linear.y;
        pubMsg.TwistStamped.twist.linear.z = thisMsg.twist.linear.z;
        pubMsg.TwistStamped.twist.angular.z = thisMsg.twist.angular.z;

        chatter_pub.publish(pubMsg);

        loop_rate.sleep();
    }
    

    return 0;
}