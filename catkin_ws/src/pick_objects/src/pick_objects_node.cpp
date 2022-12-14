#include <tuple>
#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using pose_XY_Coor_t = std::tuple<double, double>;

const int x_Coor = 0;
const int y_Coor = 1;

pose_XY_Coor_t goal_xycoor = std::make_tuple(0.0, 0.0);
bool g_flg = false;
void get_goal(const visualization_msgs::Marker::ConstPtr& msg)
{
  if ( (std::get<x_Coor>(goal_xycoor) != msg->pose.position.x) && 
       (std::get<x_Coor>(goal_xycoor) != msg->pose.position.x)) { 
    std::get<x_Coor>(goal_xycoor)  = msg->pose.position.x;
    std::get<y_Coor>(goal_xycoor)  = msg->pose.position.y;
    g_flg = true;
    ROS_INFO("Update goal %f %f", std::get<x_Coor>(goal_xycoor), std::get<y_Coor>(goal_xycoor));
  }
}


bool set_position(pose_XY_Coor_t xy_Coor, double orientation, MoveBaseClient &ac) {
	move_base_msgs::MoveBaseGoal goal;

	//Set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
#if 0	
	//Define a XY position and orientation 
	goal.target_pose.pose.position.x = std::get<y_Coor>(xy_Coor);
	goal.target_pose.pose.position.y = std::get<x_Coor>(xy_Coor) * -1.0;
	goal.target_pose.pose.orientation.w    = 1.0;
#else
	//Define a XY position and orientation 
	goal.target_pose.pose.position.x = std::get<x_Coor>(xy_Coor);
	goal.target_pose.pose.position.y = std::get<y_Coor>(xy_Coor);
	goal.target_pose.pose.orientation.w    = 1.0;
#endif
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
	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle n;
	ros::Rate r(1);

	ros::Subscriber marker_sub = n.subscribe("visualization_marker", 10, get_goal);	
	MoveBaseClient ac("move_base", true);

	//Wait for 5 seconds for move_base action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	while (!g_flg) {
		ros::spinOnce();
	}
	g_flg = false;
	if (set_position(goal_xycoor, 1.0, ac)) {
		ROS_INFO("Hooray: Pick-up Zone Reached\n");
	}
	while(!g_flg) {
		ros::spinOnce();
	}
	g_flg = false;
	if (set_position(goal_xycoor, 1.0, ac)) {
		ROS_INFO("Hooray: Drop-off Zone Reached\n");
	}
	
	return 0;	
}
