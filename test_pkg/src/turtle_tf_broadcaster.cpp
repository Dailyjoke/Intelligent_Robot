#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_tf_broadcaster");
    ros::NodeHandle nh;

    ros::param::param<std::string>("~turtle", turtle_name, "turtle1");
    ros::Subscriber sub = nh.subscribe(turtle_name + "/pose", 10, &poseCallback);

    ros::spin();
    return 0;
}
