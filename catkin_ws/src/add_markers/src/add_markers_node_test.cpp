#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

enum state {
   PICK_UP_STATE,
   HIDE_STATE,
   DROP_OFF_STATE,
};

using pose_XY_Coor_t = std::tuple<double, double>;
const int x_Coor = 0;
const int y_Coor = 1;

pose_XY_Coor_t robot_Curr_Pose;


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  state curr_State = PICK_UP_STATE;

  pose_XY_Coor_t pick_up_coor  = std::make_tuple(7.0, 5.0);
  pose_XY_Coor_t drop_off_coor = std::make_tuple(3.0, -8.0);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;


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

    switch(curr_State) {
        case PICK_UP_STATE:
        curr_State = HIDE_STATE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = std::get<y_Coor>(pick_up_coor);
        marker.pose.position.y = std::get<x_Coor>(pick_up_coor) * -1.0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        break;

        case HIDE_STATE:
        curr_State = DROP_OFF_STATE;
        marker.action = visualization_msgs::Marker::DELETE;
        break;

        case DROP_OFF_STATE:
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x =  std::get<y_Coor>(drop_off_coor);
        marker.pose.position.y =  std::get<x_Coor>(drop_off_coor) * -1.0; 
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        break; 
    }	



    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
        sleep(5);
 #if 0   
    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
#endif
    r.sleep();
  }
}
