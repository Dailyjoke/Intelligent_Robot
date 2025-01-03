#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// define MoveBaseAction action client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Navigation to Gaol
bool navigateToGoal(MoveBaseClient &ac, double x, double y, double w) {
    move_base_msgs::MoveBaseGoal goal;

    // Define target pose (Map Frame)
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("Sending goal: x=%f, y=%f, w=%f", x, y, w);
    ac.sendGoal(goal);

    // action client
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached successfully!");
        return true;
    } else {
        ROS_WARN("Failed to reach the goal.");
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_goal_navigation");

    // Create Action, and client connect to move_base
    MoveBaseClient ac("move_base", true);

    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base server");

    // 請同學依照topic 指定四個目標點並將pose進行更改
    std::vector<std::tuple<double, double, double>> goals = {
	    {-6.53114128112793, -2.053840398788452, 0.7303135602327445},     // 目標點 1
	    {-6.030522346496582, 3.3743555545806885, 0.999899903871913},       // 目標點 2
	    {-1.9796538352966309, 0.4332816004753113, 0.9999923126088655},    // 目標點 3
	    { 1.0197572708129883,  2.6753461360931396, 0.7262415664592414},  //  目標點 4
        {6.066136837005615, 0.7363654971122742, 0.7099098463145482},  //  目標點 5
        {6.026930809020996, -1.3034342527389526, 0.7161948113421561},   //  目標點 6
        {1.803402066230774, -2.9521355628967285, 0.7175668682640408}   //  目標點 7
    };

    // 依序導航至各目標點
    for (size_t i = 0; i < goals.size(); ++i) {
        double x = std::get<0>(goals[i]);
        double y = std::get<1>(goals[i]);
        double w = std::get<2>(goals[i]);

        ROS_INFO("Navigating to goal %lu/%lu", i + 1, goals.size());
        if (!navigateToGoal(ac, x, y, w)) {
            ROS_WARN("Skipping to the next goal...");
        }
    }

    ROS_INFO("Multi-goal navigation completed!");
    return 0;
}

