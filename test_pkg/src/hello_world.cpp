#include "ros/ros.h"
int main(int argc,char* argv[])
{
    ros::init(argc,argv,"hello_world_node"); //rosnode initialize
    ros::NodeHandle handler;  // create Node Handler -handler
    ROS_INFO("hello world"); //Output，hello world
    return 0;
}
