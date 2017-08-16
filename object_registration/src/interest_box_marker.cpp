#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#define X_SIZE_ARROW 0
#define Y_SIZE_ARROW 1
#define Z_SIZE_ARROW 2

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

visualization_msgs::Marker interest_box_marker;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );

  visualization_msgs::InteractiveMarker this_box_marker;
  server->get("my_marker", this_box_marker);
  std::cout << "box marker x scale:" << this_box_marker.controls[0].markers[1].scale.x << std::endl;

  // Update location of x arrow 
  geometry_msgs::Pose pose_arrow_x = feedback->pose;
  pose_arrow_x.position.x = pose_arrow_x.position.x + this_box_marker.controls[0].markers[1].scale.x/2.0;
  server->setPose("arrow_x", pose_arrow_x);

  // Update location of y arrow 
  geometry_msgs::Pose pose_arrow_y = feedback->pose;
  pose_arrow_y.position.y = pose_arrow_y.position.y + this_box_marker.controls[0].markers[1].scale.y/2.0;
  server->setPose("arrow_y", pose_arrow_y);  

  // Update location of z arrow
  geometry_msgs::Pose pose_arrow_z = feedback->pose;
  pose_arrow_z.position.z = pose_arrow_y.position.z + this_box_marker.controls[0].markers[1].scale.z/2.0;
  server->setPose("arrow_z", pose_arrow_z);  

  server->applyChanges();

}

void processFeedbackArrow(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );

  visualization_msgs::InteractiveMarker this_box_marker;
  server->get("my_marker", this_box_marker);

  // Change the size of the box depending on the arrow type
  if ( (feedback->marker_name).compare("arrow_x") == 0 ){
    std::cout << "box marker x scale:" << this_box_marker.controls[0].markers[1].scale.x << std::endl;
    this_box_marker.controls[0].markers[1].scale.x = 2.0*(feedback->pose.position.x - this_box_marker.pose.position.x);    
  }else if( (feedback->marker_name).compare("arrow_y") == 0) {
    std::cout << "box marker y scale:" << this_box_marker.controls[0].markers[1].scale.y << std::endl;
    this_box_marker.controls[0].markers[1].scale.y = 2.0*(feedback->pose.position.y - this_box_marker.pose.position.y);
  }else if( (feedback->marker_name).compare("arrow_z") == 0) {
    std::cout << "box marker z scale:" << this_box_marker.controls[0].markers[1].scale.z << std::endl;
    this_box_marker.controls[0].markers[1].scale.z = 2.0*(feedback->pose.position.z - this_box_marker.pose.position.z);
  }

  server->insert(this_box_marker);
  server->applyChanges();

}

void processFeedbackArrow_y(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );

  visualization_msgs::InteractiveMarker this_box_marker;
  server->get("my_marker", this_box_marker);
  std::cout << "box marker y scale:" << this_box_marker.controls[0].markers[1].scale.y << std::endl;
  this_box_marker.controls[0].markers[1].scale.y = 2.0*(feedback->pose.position.y - this_box_marker.pose.position.y);

  server->insert(this_box_marker);
  server->applyChanges();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_registration_interest_box");

  // create an interactive marker server on the topic namespace simple_marker
  //interactive_markers::InteractiveMarkerServer server("simple_marker");

  server.reset( new interactive_markers::InteractiveMarkerServer("simple_marker","",false) );

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
//  int_marker.description = "Simple 1-DOF Control";

  // create a grey box marker
  visualization_msgs::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.scale.x = 0.05;
  sphere_marker.scale.y = 0.05;
  sphere_marker.scale.z = 0.05;
  sphere_marker.color.r = 0.5;
  sphere_marker.color.g = 0.5;
  sphere_marker.color.b = 0.5;
  sphere_marker.color.a = 0.5;


  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 1.0;
  box_marker.scale.y = 1.0;
  box_marker.scale.z = 1.0;
  box_marker.color.r = 0.0;
  box_marker.color.g = 0.0;
  box_marker.color.b = 1.0;
  box_marker.color.a = 0.5;


  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl sphere_control;
  sphere_control.always_visible = true;
  sphere_control.markers.push_back( sphere_marker );
  sphere_control.markers.push_back(box_marker);

  // add the control to the interactive marker
  sphere_control.name = "click_and_move_3d";
  sphere_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  int_marker.controls.push_back( sphere_control );


  // Create x size arrow interactive marker
  visualization_msgs::InteractiveMarker int_marker_arrow_x;
  int_marker_arrow_x.pose.position.x = box_marker.scale.x/2.0;
  int_marker_arrow_x.header.frame_id = "base_link";
  int_marker_arrow_x.header.stamp=ros::Time::now();
  int_marker_arrow_x.name = "arrow_x";

  visualization_msgs::Marker arrow_x;
  arrow_x.pose.orientation.x = 0.0;   arrow_x.pose.orientation.y = 0.0;   arrow_x.pose.orientation.z = 0.0;  
  arrow_x.pose.orientation.w = 1.0;
  arrow_x.type = visualization_msgs::Marker::ARROW;
  arrow_x.scale.x = 0.25;
  arrow_x.scale.y = 0.1;
  arrow_x.scale.z = 0.1;

  arrow_x.color.r = 0.0; 
  arrow_x.color.g = 1.0;
  arrow_x.color.b = 0.0;
  arrow_x.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl size_arrow_x;
  size_arrow_x.always_visible = true;
  size_arrow_x.name = "move_x";
  size_arrow_x.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  size_arrow_x.markers.push_back(arrow_x);
  int_marker_arrow_x.controls.push_back(size_arrow_x);  



  // Create y size arrow interactive marker
  visualization_msgs::InteractiveMarker int_marker_arrow_y;
  int_marker_arrow_y.pose.position.y = box_marker.scale.y/2.0;
  int_marker_arrow_y.header.frame_id = "base_link";
  int_marker_arrow_y.header.stamp=ros::Time::now();
  int_marker_arrow_y.name = "arrow_y";

  visualization_msgs::Marker arrow_y;
  arrow_y.pose.orientation.x = 0.0;   
  arrow_y.pose.orientation.y = 0.0;   
  arrow_y.pose.orientation.z = 1.0;
  arrow_y.pose.orientation.w = 1.0;  

  arrow_y.type = visualization_msgs::Marker::ARROW;
  arrow_y.scale.x = 0.25;
  arrow_y.scale.y = 0.1;
  arrow_y.scale.z = 0.1;

  arrow_y.color.r = 0.0; 
  arrow_y.color.g = 1.0;
  arrow_y.color.b = 0.0;
  arrow_y.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl size_arrow_y;
  size_arrow_y.always_visible = true;
  size_arrow_y.name = "move_y";
  size_arrow_y.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  size_arrow_y.orientation = arrow_y.pose.orientation;

  size_arrow_y.markers.push_back(arrow_y);
  int_marker_arrow_y.controls.push_back(size_arrow_y);    


  // Create z size arrow interactive marker
  visualization_msgs::InteractiveMarker int_marker_arrow_z;
  int_marker_arrow_z.pose.position.z = box_marker.scale.z/2.0;
  int_marker_arrow_z.header.frame_id = "base_link";
  int_marker_arrow_z.header.stamp=ros::Time::now();
  int_marker_arrow_z.name = "arrow_z";

  visualization_msgs::Marker arrow_z;
  arrow_z.pose.orientation.x = 0.0;   
  arrow_z.pose.orientation.y = 1.0;   
  arrow_z.pose.orientation.z = 0.0;  
  arrow_z.pose.orientation.w = -1;
  arrow_z.type = visualization_msgs::Marker::ARROW;
  arrow_z.scale.x = 0.25;
  arrow_z.scale.y = 0.1;
  arrow_z.scale.z = 0.1;

  arrow_z.color.r = 0.0; 
  arrow_z.color.g = 1.0;
  arrow_z.color.b = 0.0;
  arrow_z.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl size_arrow_z;
  size_arrow_z.always_visible = true;
  size_arrow_z.name = "move_z";
  size_arrow_z.orientation = arrow_z.pose.orientation;
  size_arrow_z.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  size_arrow_z.markers.push_back(arrow_z);
  int_marker_arrow_z.controls.push_back(size_arrow_z);    


  server->insert(int_marker, &processFeedback);
  server->insert(int_marker_arrow_x, &processFeedbackArrow);  
  server->insert(int_marker_arrow_y, &processFeedbackArrow);    
  server->insert(int_marker_arrow_z, &processFeedbackArrow);      

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
