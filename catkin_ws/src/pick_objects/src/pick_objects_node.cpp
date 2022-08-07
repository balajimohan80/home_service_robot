#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv) {
	//Initialize the node and name of the node is "simple_naviagation_goals"	
	ros::init(argc, argv, "simple_navigation_goals");

	//Tell the action client that spin a thread by default
	MoveBaseClient ac("move_base", true);

	//Wait for 5 seconds for move_base action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}
				
	move_base_msgs::MoveBaseGoal end_goal;

	//Set up the feame parameters
	end_goal.target_pose.header.frame_id = "map";
	end_goal.target_pose.header.stamp = ros::Time::now();

	//Define a position and orientation for the 
	end_goal.target_pose.pose.position.x = 5.0;
	end_goal.target_pose.pose.orientation.w    = 1.0;

	//Send the goal Position and orientation for the robot to reach
	ROS_INFO("Sending goal");
	ac.sendGoal(end_goal);	

	//Wait an infinite time for the results
	ac.waitForResult();

	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved 1 meter forward");
	else
		ROS_INFO("The base failed to move forward 1 meter for some reason");


	return 0;	
}
