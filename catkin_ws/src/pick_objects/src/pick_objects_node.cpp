#include <tuple>
#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using end_Goal_XY_Coor_t = std::tuple<double, double>;

const int x_Coor = 0;
const int y_Coor = 1;
const end_Goal_XY_Coor_t end_goal_xycoor = std::make_tuple(5.0, 2.0);

void get_robot_pose(const nav_msgs::Odometry::ConstPtr& msg) {
	double dx = std::abs(std::get<x_Coor>(end_goal_xycoor)) - 
	            std::abs(msg->pose.pose.position.x);
	double dy = std::abs(std::get<y_Coor>(end_goal_xycoor)) - 
	            std::abs(msg->pose.pose.position.y);
	dx *= dx;
	dy *= dy;
	double dist = std::sqrt(dx + dy);
	std::cout << "x: " << msg->pose.pose.position.x << "y: " << msg->pose.pose.position.y <<  " DISTANCE: " << dist << "\n";
}

int main(int argc, char **argv) {
	//Initialize the node and name of the node is "simple_naviagation_goals"	
	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Subscriber pose_sub = n.subscribe("odom", 10, get_robot_pose);

	//Tell the action client that spin a thread by default
	MoveBaseClient ac("move_base", true);

	//Wait for 5 seconds for move_base action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ros::spinOnce();
	move_base_msgs::MoveBaseGoal end_goal;

	//Set up the feame parameters
	end_goal.target_pose.header.frame_id = "map";
	end_goal.target_pose.header.stamp = ros::Time::now();

	//Define a position and orientation for the 
	end_goal.target_pose.pose.position.x = std::get<x_Coor>(end_goal_xycoor);
	end_goal.target_pose.pose.position.y = std::get<y_Coor>(end_goal_xycoor);
	end_goal.target_pose.pose.orientation.w    = 1.0;
	//Send the goal Position and orientation for the robot to reach
	ROS_INFO("Sending goal");
	ac.sendGoal(end_goal);	

	//Wait an infinite time for the results
	while (!ac.waitForResult(ros::Duration(1.0))) {
		ros::spinOnce();
	}

	ros::spinOnce();
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved 1 meter forward");
	else
		ROS_INFO("The base failed to move forward 1 meter for some reason");

	ros::spinOnce();
	return 0;	
}
