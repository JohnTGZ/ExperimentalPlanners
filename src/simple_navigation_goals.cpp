#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void setGoal(double wx, double wy)
{
    MoveBaseClient ac("move_base", true); //spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = wx;
    goal.target_pose.pose.position.y = wy;
    goal.target_pose.pose.position.z = 1.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Success moving to goal( x: %f, y: %f )", wx, wy);
    else
        ROS_INFO("Failed");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    setGoal(1.75, 1.25);
    setGoal(-5.5, -2.8);
    setGoal(0.29, 5.58);

    return 0;

}