#include <iostream>
#include <cmath>
#include <tuple>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


enum state {
   PICK_UP_STATE,
   HIDE_STATE,
   DROP_OFF_STATE,
   END_STATE,
};

using pose_XY_Coor_t = std::tuple<double, double>;
const int x_Coor = 0;
const int y_Coor = 1;

pose_XY_Coor_t robot_Curr_Pose;

void get_robot_pose(const nav_msgs::Odometry::ConstPtr& msg) {
	std::get<x_Coor>(robot_Curr_Pose) = msg->pose.pose.position.x;
	std::get<y_Coor>(robot_Curr_Pose) = msg->pose.pose.position.y;
}

double compute_Distance(pose_XY_Coor_t src, pose_XY_Coor_t dst) {
	double dx = std::abs(std::get<x_Coor>(src)) - 
	            std::abs(std::get<x_Coor>(dst));

	double dy = std::abs(std::get<y_Coor>(src)) -
	            std::abs(std::get<y_Coor>(dst));
	dx *= dx;
	dy *= dy;
	return std::sqrt(dx + dy);
}

void publish_visulaization_marker(pose_XY_Coor_t disp_xy, ros::Publisher &pub, int32_t action) {
    visualization_msgs::Marker marker;
	
	
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;


    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker.action = action;
    marker.pose.position.x = std::get<y_Coor>(disp_xy);
    marker.pose.position.y = std::get<x_Coor>(disp_xy) * -1.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Publish the marker
    while (pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    pub.publish(marker);
    return;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  pose_XY_Coor_t pick_up_coor  = std::make_tuple(7.0, 5.0);
  pose_XY_Coor_t drop_off_coor = std::make_tuple(3.0, -8.0); 	

  ros::Subscriber pose_sub = n.subscribe("odom", 10, get_robot_pose);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::spinOnce();
  state next_State = PICK_UP_STATE;
  bool brk_flg = true;
  while (ros::ok() && brk_flg)
  {

    switch(next_State) {
        case PICK_UP_STATE:
        next_State = HIDE_STATE;
	publish_visulaization_marker(pick_up_coor, marker_pub, visualization_msgs::Marker::ADD);
	ros::spinOnce();
	while (0.3 < compute_Distance(robot_Curr_Pose, pick_up_coor)) {
		ros::spinOnce();
	} 
        break;

        case HIDE_STATE:
        next_State = DROP_OFF_STATE;
	publish_visulaization_marker(pick_up_coor, marker_pub, visualization_msgs::Marker::DELETE);
        sleep(5);
	break;

        case DROP_OFF_STATE:
	publish_visulaization_marker(drop_off_coor, marker_pub, visualization_msgs::Marker::DELETE);
        ros::spinOnce();
	while (0.3 < compute_Distance(robot_Curr_Pose, drop_off_coor)) {
		ros::spinOnce();
	}
	publish_visulaization_marker(drop_off_coor, marker_pub, visualization_msgs::Marker::ADD);
	next_State = END_STATE;	
	break; 

	case END_STATE:
	brk_flg = false;
	break;
    }	

    r.sleep();
  }
  return 0;
}
