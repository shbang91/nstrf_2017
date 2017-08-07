/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <std_msgs/String.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


#include <math.h>

#define LEFT_HAND 0
#define RIGHT_HAND 1

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%


int hand_to_use = RIGHT_HAND;


// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "world", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "world", "rotating_frame"));

  counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server->setPose( feedback->marker_name, pose );
  server->applyChanges();
}
// %EndTag(alignMarker)%

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, tf::Quaternion quat_start, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  tf::quaternionTFToMsg   (quat_start, int_marker.pose.orientation); 
  int_marker.scale = 0.25;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

// %Tag(RandomDof)%
void makeRandomDofMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "6dof_random_axes";
  int_marker.description = "6-DOF\n(Arbitrary Axes)";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  for ( int i=0; i<3; i++ )
  {
    control.orientation.w = rand(-1,1);
    control.orientation.x = rand(-1,1);
    control.orientation.y = rand(-1,1);
    control.orientation.z = rand(-1,1);
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(RandomDof)%


// %Tag(ViewFacing)%
void makeViewFacingMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "view_facing";
  int_marker.description = "View Facing 6-DOF";

  InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.name = "rotate";

  int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;

  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(ViewFacing)%


// %Tag(Quadrocopter)%
void makeQuadrocopterMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "quadrocopter";
  int_marker.description = "Quadrocopter";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Quadrocopter)%

// %Tag(ChessPiece)%
void makeChessPieceMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}
// %EndTag(ChessPiece)%

// %Tag(PanTilt)%
void makePanTiltMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "pan_tilt";
  int_marker.description = "Pan / Tilt";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(PanTilt)%

// %Tag(Menu)%
void makeMenuMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply( *server, int_marker.name );
}
// %EndTag(Menu)%

// %Tag(Button)%
void makeButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Button)%

// %Tag(Moving)%
void makeMovingMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%
bool getFramePose(const std::string& target_frame,  geometry_msgs::Pose& frame_pose){
    tf::TransformListener       tf_listener;  
    int attempts = 0;
    while (ros::ok()){
      if (attempts == 10){
        ROS_ERROR("Cannot Get Frame Pose after 10 attempts");
        break;
      }
      tf::StampedTransform world_to_rpalm_transform;
      std::string origin_frame; origin_frame = "world";
      try{
      // Look up transform

        tf_listener.lookupTransform(origin_frame, target_frame, ros::Time(0), world_to_rpalm_transform);
        std::cout << "Frame ID:" << world_to_rpalm_transform.frame_id_ << std::endl;
        std::cout << "Child ID:" << world_to_rpalm_transform.child_frame_id_ << std::endl;
        std::cout << "Trans (x,y,z) = " << world_to_rpalm_transform.getOrigin().getX() << " " << world_to_rpalm_transform.getOrigin().getY() << " " << world_to_rpalm_transform.getOrigin().getZ() << std::endl;
        
        tf::Quaternion q;
        q = world_to_rpalm_transform.getRotation(); 
        q.normalize();
        geometry_msgs::Quaternion quat_msg;
        tf::quaternionTFToMsg (q, quat_msg);

        std::cout << "Quat  (x,y,z,w) = " << quat_msg.x << " " << quat_msg.y << " " << quat_msg.z << " " << quat_msg.w << std::endl;        
        // Use world_to_rpalm_transform

        frame_pose.position.x =  world_to_rpalm_transform.getOrigin().getX();
        frame_pose.position.y =  world_to_rpalm_transform.getOrigin().getY();          
        frame_pose.position.z =  world_to_rpalm_transform.getOrigin().getZ();
        frame_pose.orientation = quat_msg;

        return true;

      }
      //keep trying until we get the transform
      catch (tf::TransformException ex){
        ROS_WARN_THROTTLE(2,"%s",ex.what());
        ROS_WARN_THROTTLE(2, "   Waiting for tf to transform world to end effector frame. Trying again");
        ros::Duration(1.0).sleep();
        attempts = attempts + 1;

      }
    }
  return false;
}

//void setIM_pose_to_current_robot_state
void setIM_hand_pose_to_current_robot_state(){
    std::string im_name; im_name = "simple_6dof_MOVE_ROTATE_3D";
    geometry_msgs::Pose des_pose;  
    if (hand_to_use == LEFT_HAND){
        if  (getFramePose("leftPalm", des_pose)){
            server->setPose( im_name, des_pose);
            server->applyChanges();
        }else{
            ROS_ERROR("Failed to change interactive marker pose");
        }
    }else{
        if  (getFramePose("rightPalm", des_pose)){
            server->setPose( im_name, des_pose);
            server->applyChanges();
        }else{
            ROS_ERROR("Failed to change interactive marker pose");
        }
    }
}


void callback_operator(const std_msgs::StringConstPtr& msg){
    std::string re_init_markers; re_init_markers = "re_init_markers";
    std::string use_right_hand; use_right_hand = "use_right_hand";    
    std::string use_left_hand; use_left_hand = "use_left_hand";    

    if (re_init_markers.compare(msg->data) == 0){
        setIM_hand_pose_to_current_robot_state();

    }else if(use_right_hand.compare(msg->data) == 0){
        ROS_INFO("IM callback. Will now use right hand");
        hand_to_use = RIGHT_HAND;
        setIM_hand_pose_to_current_robot_state();
    }else if(use_left_hand.compare(msg->data) == 0){
        ROS_INFO("IM callback. Will now use left hand");      
        hand_to_use = LEFT_HAND;
        setIM_hand_pose_to_current_robot_state();        
    }

}



// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;
  ros::Subscriber operator_command_sub;

  operator_command_sub = n.subscribe<std_msgs::String>("val_logic_manager/operator_command", 1, callback_operator);

  // create a timer to update the published transforms
  //ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

  ros::Duration(0.1).sleep();

  menu_handler.insert( "First Entry", &processFeedback );
  menu_handler.insert( "Second Entry", &processFeedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

  tf::Vector3 position;
 /* position = tf::Vector3(-3, 3, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
  position = tf::Vector3( 0, 3, 0);
  make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
  position = tf::Vector3( 3, 3, 0);
  makeRandomDofMarker( position );
  position = tf::Vector3(-3, 0, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::ROTATE_3D, position, false );*/

/*  position = tf::Vector3( 0.3695, -0.147, 0.938);
  tf::Quaternion q_start(-0.0308, 0.096, 0.883, 0.458);*/

    position = tf::Vector3( 0.586061835289, -0.193872511387, 1.14380383492);
    tf::Quaternion q_start(0.423302135094, -0.554713968345,  0.473368349139, 0.537615204056);

    geometry_msgs::Pose des_pose;
    if  (getFramePose("rightPalm", des_pose)){
        position.setX(des_pose.position.x);
        position.setY(des_pose.position.y);
        position.setZ(des_pose.position.z);                

        tf::Quaternion q_start_pose(des_pose.orientation.x, des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
        q_start = q_start_pose;
    }        

    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, q_start, true );
/*  position = tf::Vector3( 3, 0, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, false );
  position = tf::Vector3(-3,-3, 0);
  makeViewFacingMarker( position );
  position = tf::Vector3( 0,-3, 0);
  makeQuadrocopterMarker( position );
  position = tf::Vector3( 3,-3, 0);
  makeChessPieceMarker( position );
  position = tf::Vector3(-3,-6, 0);
  makePanTiltMarker( position );
  position = tf::Vector3( 0,-6, 0);
  makeMovingMarker( position );
  position = tf::Vector3( 3,-6, 0);
  makeMenuMarker( position );
  position = tf::Vector3( 0,-9, 0);
  makeButtonMarker( position );*/

    server->applyChanges();
    ros::spin();
    server.reset();
}
// %EndTag(main)%


