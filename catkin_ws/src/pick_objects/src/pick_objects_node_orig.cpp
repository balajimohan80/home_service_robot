#include <tuple>
#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using goal_XY_Coor_t = std::tuple<double, double>;

const int x_Coor = 0;
const int y_Coor = 1;

goal_XY_Coor_t goal_xycoor;

bool distance_reached = false;
void get_robot_pose(const nav_msgs::Odometry::ConstPtr& msg) {
	double dx = std::abs(std::get<x_Coor>(goal_xycoor)) - 
	            std::abs(msg->pose.pose.position.x);
	double dy = std::abs(std::get<y_Coor>(goal_xycoor)) - 
	            std::abs(msg->pose.pose.position.y);
	dx *= dx;
	dy *= dy;
	distance_reached = 0.3 > std::sqrt(dx + dy) ? true : false;
}


bool set_position(goal_XY_Coor_t xy_Coor, double orientation, MoveBaseClient &ac) {
	move_base_msgs::MoveBaseGoal goal;

	//Set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	
	//Define a XY position and orientation 
	goal.target_pose.pose.position.x = std::get<y_Coor>(xy_Coor);
	goal.target_pose.pose.position.y = std::get<x_Coor>(xy_Coor) * -1.0;
	goal.target_pose.pose.orientation.w    = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();
	bool goal_Reached = false;
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		goal_Reached = true;
	}
	return goal_Reached;
}

int main(int argc, char **argv) {
	//Initialize the node and name of the node is "simple_naviagation_goals"	
	ros::init(argc, argv, "pick_objects");
	ros::NodeHandle n;
	ros::Rate r(1);

	MoveBaseClient ac("move_base", true);

	//Wait for 5 seconds for move_base action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	const goal_XY_Coor_t pick_up_xy_Coor  = std::make_tuple(7.0, 5.0);
	const goal_XY_Coor_t drop_off_xy_Coor = std::make_tuple(3.0, -8.0);

	goal_xycoor = pick_up_xy_Coor; 
	ros::Subscriber pose_sub = n.subscribe("odom", 10, get_robot_pose);
	
	if (set_position(pick_up_xy_Coor, 1.0, ac)) {
		ROS_INFO("Hooray: Pick-up Zone Reached\n");
	}
	for (int i = 0; i < 5; i++) {
		ros::spinOnce();
		sleep(1);
	}

	goal_xycoor = drop_off_xy_Coor;
	distance_reached = 0;
	if (set_position(drop_off_xy_Coor, 1.0, ac)) {
		ROS_INFO("Hooray: Drop-off Zone Reached\n");
	}
	
	for (int i = 0; i < 5; i++) {
		ros::spinOnce();
		sleep(2);
	}
	return 0;	
}
