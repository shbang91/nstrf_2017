#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>


#define MARKER_FRAME "world"
#define MARKER_NAME "interestBox"

#define X_SIZE_ARROW_1 "arrow_x"
#define Y_SIZE_ARROW_1 "arrow_y"
#define Z_SIZE_ARROW_1 "arrow_z"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
visualization_msgs::Marker interest_box_marker;

void createBoxMarker(visualization_msgs::InteractiveMarker& int_marker,
                     visualization_msgs::Marker& box_marker_copy){
  int_marker.header.frame_id = MARKER_FRAME;
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = MARKER_NAME;
  int_marker.description = "draggable_box";

  // create a sphere marker to represent the center
  visualization_msgs::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.scale.x = 0.05;  sphere_marker.scale.y = 0.05;  sphere_marker.scale.z = 0.05;
  sphere_marker.color.r = 0.5;   sphere_marker.color.g = 0.5;   sphere_marker.color.b = 0.5;   sphere_marker.color.a = 0.5;

  //create a box marker to represent the bounding box
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 1.0;  box_marker.scale.y = 1.0; box_marker.scale.z = 1.0;
  box_marker.color.r = 0.0;  box_marker.color.g = 0.0; box_marker.color.b = 1.0; box_marker.color.a = 0.5;

  // Copy this marker for initialization purposes
  box_marker_copy = box_marker;

  // create a controller to translate the box in 3D by dragging it
  visualization_msgs::InteractiveMarkerControl box_translate_control;
  box_translate_control.name = "click_and_move_3d";
  box_translate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  box_translate_control.always_visible = true;

  // Use these markers to represent the clickable objects
  box_translate_control.markers.push_back(sphere_marker);
  box_translate_control.markers.push_back(box_marker);

  // add the control to the interactive marker
  int_marker.controls.push_back( box_translate_control );
}

void createArrowMarker(const std::string arrow_name, const float box_size_in_dimension,
                       visualization_msgs::InteractiveMarker& int_marker_arrow){
  // Initialize the Markers we need
  visualization_msgs::Marker arrow_marker;
  visualization_msgs::InteractiveMarkerControl size_arrow_control;

  // Initialize the Pose of the Arrow
  int_marker_arrow.header.frame_id = MARKER_FRAME;
  int_marker_arrow.header.stamp = ros::Time::now();

  // Initialize the visualization of the arrow
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.scale.x = 0.25; arrow_marker.scale.y = 0.1;  arrow_marker.scale.z = 0.1;
  arrow_marker.color.r = 0.0;  arrow_marker.color.g = 1.0;  arrow_marker.color.b = 0.0; arrow_marker.color.a = 0.5;

  // Set the pose, orientation and names
  if (arrow_name.compare(X_SIZE_ARROW_1) == 0){
      // Set initial interactive marker pose
      int_marker_arrow.pose.position.x = box_size_in_dimension/2.0;       
      // Set Arrow Orientation      
      arrow_marker.pose.orientation.x = 0.0;   
      arrow_marker.pose.orientation.y = 0.0;   
      arrow_marker.pose.orientation.z = 0.0;  
      arrow_marker.pose.orientation.w = 1.0;

      int_marker_arrow.name = X_SIZE_ARROW_1;
      size_arrow_control.name = "move_x";      
  }else if ( arrow_name.compare(Y_SIZE_ARROW_1) == 0) {
      // Set initial interactive marker pose    
      int_marker_arrow.pose.position.y = box_size_in_dimension/2.0;
      
      // Set Arrow Orientation
      arrow_marker.pose.orientation.x = 0.0;   
      arrow_marker.pose.orientation.y = 0.0;   
      arrow_marker.pose.orientation.z = 1.0;
      arrow_marker.pose.orientation.w = 1.0;      

      int_marker_arrow.name = Y_SIZE_ARROW_1;
      size_arrow_control.name = "move_y";
  }else if ( arrow_name.compare(Z_SIZE_ARROW_1) == 0) {
      // Set initial interactive marker pose    
      int_marker_arrow.pose.position.z = box_size_in_dimension/2.0;
      
      // Set Arrow Orientation
      arrow_marker.pose.orientation.x = 0.0;   
      arrow_marker.pose.orientation.y = 1.0;   
      arrow_marker.pose.orientation.z = 0.0;
      arrow_marker.pose.orientation.w = -1.0;      

      int_marker_arrow.name = Z_SIZE_ARROW_1;
      size_arrow_control.name = "move_z";
  }
  // Set the orientation of the control direction
  size_arrow_control.orientation = arrow_marker.pose.orientation;

  // Specify the control properties
  size_arrow_control.always_visible = true;
  size_arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // Use the arrow marker as the draggable object
  size_arrow_control.markers.push_back(arrow_marker);

  // Add the control to the arrow interactive marker
  int_marker_arrow.controls.push_back(size_arrow_control);    
}

void updateArrowLocations(const geometry_msgs::Pose box_pose, 
                          const visualization_msgs::Marker box_marker){
  // Update location of x arrow 
  geometry_msgs::Pose pose_arrow_x = box_pose;
  pose_arrow_x.position.x = pose_arrow_x.position.x + box_marker.scale.x/2.0;
  server->setPose(X_SIZE_ARROW_1, pose_arrow_x);

  // Update location of y arrow 
  geometry_msgs::Pose pose_arrow_y = box_pose;
  pose_arrow_y.position.y = pose_arrow_y.position.y + box_marker.scale.y/2.0;
  server->setPose(Y_SIZE_ARROW_1, pose_arrow_y);  

  // Update location of z arrow
  geometry_msgs::Pose pose_arrow_z = box_pose;
  pose_arrow_z.position.z = pose_arrow_y.position.z + box_marker.scale.z/2.0;
  server->setPose(Z_SIZE_ARROW_1, pose_arrow_z);

  server->applyChanges();
}


void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y  << ", " << feedback->pose.position.z );

  visualization_msgs::InteractiveMarker int_box_marker;
  visualization_msgs::Marker box_marker_copy;

  server->get(MARKER_NAME, int_box_marker);
  box_marker_copy = int_box_marker.controls[0].markers[1];

  updateArrowLocations(feedback->pose, box_marker_copy);
}

void processFeedbackArrow(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO_STREAM( feedback->marker_name << " is now at "<< feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );

  visualization_msgs::InteractiveMarker int_box_marker;
  visualization_msgs::Marker box_marker_copy;

  // Get a copy of the Interactive box marker
  server->get(MARKER_NAME, int_box_marker);
  box_marker_copy = int_box_marker.controls[0].markers[1];  

  // Change the size of the box depending on the arrow type
  if ( (feedback->marker_name).compare(X_SIZE_ARROW_1) == 0 ){
    std::cout << "box marker x scale:" << int_box_marker.controls[0].markers[1].scale.x << std::endl;
    box_marker_copy.scale.x = 2.0*(feedback->pose.position.x - int_box_marker.pose.position.x);    
  }else if( (feedback->marker_name).compare(Y_SIZE_ARROW_1) == 0) {
    std::cout << "box marker y scale:" << int_box_marker.controls[0].markers[1].scale.y << std::endl;
    box_marker_copy.scale.y = 2.0*(feedback->pose.position.y - int_box_marker.pose.position.y);
  }else if( (feedback->marker_name).compare(Z_SIZE_ARROW_1) == 0) {
    std::cout << "box marker z scale:" << int_box_marker.controls[0].markers[1].scale.z << std::endl;
    box_marker_copy.scale.z = 2.0*(feedback->pose.position.z - int_box_marker.pose.position.z);
  }

  // Update the interactive box marker with a new size
  int_box_marker.controls[0].markers[1] = box_marker_copy;
  server->insert(int_box_marker);
  server->applyChanges();
}


void getCloudinBox(){ 
}

void getRelativePose(std::string hand_side){ 
}

void storeCloudandRelativePose(){
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_registration_interest_box");
  // Set the interactive marker server on the topic namespace simple_marker
  server.reset( new interactive_markers::InteractiveMarkerServer("simple_marker","",false) );

  // Create the Interactive Box Marker which will indicate the region of interest
  visualization_msgs::InteractiveMarker int_marker;
  visualization_msgs::Marker box_marker_copy;
  createBoxMarker(int_marker, box_marker_copy);

  // Create the Interactive Arrow Markers which changes the size of the box
  visualization_msgs::InteractiveMarker int_marker_arrow_x;
  visualization_msgs::InteractiveMarker int_marker_arrow_y;
  visualization_msgs::InteractiveMarker int_marker_arrow_z;
  createArrowMarker(X_SIZE_ARROW_1, box_marker_copy.scale.x, int_marker_arrow_x);
  createArrowMarker(Y_SIZE_ARROW_1, box_marker_copy.scale.y, int_marker_arrow_y);
  createArrowMarker(Z_SIZE_ARROW_1, box_marker_copy.scale.z, int_marker_arrow_z);  

  // Insert to the Server
  server->insert(int_marker, &processFeedback);
  server->insert(int_marker_arrow_x, &processFeedbackArrow);  
  server->insert(int_marker_arrow_y, &processFeedbackArrow);    
  server->insert(int_marker_arrow_z, &processFeedbackArrow);      

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();
}
