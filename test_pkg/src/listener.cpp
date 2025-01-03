#include "ros/ros.h"
#include "std_msgs/String.h"

/*
Callback function :event trigger，例如訊息傳遞時，此函數將自動被呼叫。
當ROS 收到來自"chatter" 話題的訊息時，這個函數就會被觸發。
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard:[%s]",msg->data.c_str());
}

int main(int argc,char **argv){
    ros::init(argc,argv,"listener");
    ros::NodeHandle n;
    ros::Subscriber sub;            //ros Subscriber
    sub = n.subscribe("chatter",
                      1000, 
                      chatterCallback);
    ros::spin();
    return 0;
    
}