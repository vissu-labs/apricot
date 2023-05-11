#include <ros/ros.h>
#include <iostream>
// #include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
// #include <move_base_msgs/MoveBaseActionFeedback.h>
// #include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_publisher");
    std::vector<std::vector<float>> goals{{0.723, 5.177}, {4.6, 2.183}, {-0.603, -1.713}, {-3.49, 2.27}};
    Client client("/move_base", true);
    client.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.orientation.w = 1.0;
    size_t index{0};

    while (ros::ok())
    {
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goals[index][0];
        goal.target_pose.pose.position.y = goals[index][1];
        client.sendGoal(goal);
        client.waitForResult();
        ros::Rate r(0.5);
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The goal is succeded!");
        }
        else
        {
            ROS_INFO("The goal not reached moving to next goal");
        }
        r.sleep();
        if (index >= goals.size() - 1)
        {
            index = 0;
            break;
        }
        else
        {
            ++index;
        }
    }
    return 0;
}