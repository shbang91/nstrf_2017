// This code tests the openpose rosservice node by loading an image and calling the service.#include <ros/ros.h>
#include <ros/ros.h>
#include <signal.h>

// Include ROS message types
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "val_ik_msgs/BodyPositionConstraint.h"
#include "val_ik_msgs/BodyQuaternionConstraint.h"
#include "val_ik_msgs/JointPositionConstraint.h"
#include "val_ik_msgs/RobotJointStates.h"

// Include ROS Service
#include "val_ik/DrakeIKVal.h"

#include <tf/transform_broadcaster.h>

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


void init_IK_joint_names(std::vector<std::string> &drake_floating_joint_names, std::vector<std::string>  &drake_body_joint_names){
    drake_floating_joint_names.push_back("base_x");  
    drake_floating_joint_names.push_back("base_y"); 
    drake_floating_joint_names.push_back("base_z");
    drake_floating_joint_names.push_back("base_roll"); 
    drake_floating_joint_names.push_back("base_pitch"); 
    drake_floating_joint_names.push_back("base_yaw");

    drake_body_joint_names.push_back("torsoYaw");
    drake_body_joint_names.push_back("torsoPitch");
    drake_body_joint_names.push_back("torsoRoll");
    drake_body_joint_names.push_back("lowerNeckPitch");    
    drake_body_joint_names.push_back("rightShoulderPitch");
    drake_body_joint_names.push_back("rightShoulderRoll");
    drake_body_joint_names.push_back("rightShoulderYaw");
    drake_body_joint_names.push_back("rightElbowPitch");
    drake_body_joint_names.push_back("rightForearmYaw");
    drake_body_joint_names.push_back("rightWristRoll");
    drake_body_joint_names.push_back("rightWristPItch");
    drake_body_joint_names.push_back("leftShoulderPitch");
    drake_body_joint_names.push_back("leftShoulderRoll");
    drake_body_joint_names.push_back("leftShoulderYaw");
    drake_body_joint_names.push_back("leftElbowPitch");
    drake_body_joint_names.push_back("leftForearmYaw");
    drake_body_joint_names.push_back("leftWristRoll");
    drake_body_joint_names.push_back("LeftWristPitch");
    drake_body_joint_names.push_back("rightHipYaw");
    drake_body_joint_names.push_back("rightHipRoll");
    drake_body_joint_names.push_back("rightHipPitch");
    drake_body_joint_names.push_back("rightKneePitch");
    drake_body_joint_names.push_back("rightAnklePitch");
    drake_body_joint_names.push_back("rightAnkleRoll");
    drake_body_joint_names.push_back("leftHipYaw");
    drake_body_joint_names.push_back("leftHipRoll");
    drake_body_joint_names.push_back("leftHipPitch");
    drake_body_joint_names.push_back("leftKneePitch");
    drake_body_joint_names.push_back("leftAnklePitch");
    drake_body_joint_names.push_back("leftAnkleRoll");  
}

void init_IK_positions(const std::vector<std::string>  &drake_floating_joint_names, 
                       const std::vector<std::string>  &drake_body_joint_names,
                       std::vector<float>              &init_drake_floating_joint_pos,
                       std::vector<float>              &init_drake_body_joint_pos){


    for(size_t i = 0; i < drake_floating_joint_names.size(); i++){
        init_drake_floating_joint_pos.push_back(0.0);
    }
    for(size_t i = 0; i < drake_body_joint_names.size(); i++){
        init_drake_body_joint_pos.push_back(0.0);
    }    
}

void define_IK_init_positions_test(std::vector<float>   &init_drake_floating_joint_pos,
                                   std::vector<float>  &init_drake_body_joint_pos){
    init_drake_floating_joint_pos[0] = 0.0;   //0 base_x
    init_drake_floating_joint_pos[1] = 0.0;   //1 base_y
    init_drake_floating_joint_pos[2] = 1.025; //2 base_z
    init_drake_floating_joint_pos[3] = 0.0;   //3 base_roll
    init_drake_floating_joint_pos[4] = 0.0;   //4 base_pitch
    init_drake_floating_joint_pos[5] = 0.0;   //5 base_yaw    

    init_drake_body_joint_pos[0] = 0.0; // 6 torsoYaw
    init_drake_body_joint_pos[1] = 0.0; // 7 torsoPitch
    init_drake_body_joint_pos[2] = 0.0; // 8 torsoRoll    
    init_drake_body_joint_pos[3] = 0.0; // 9 lowerNeckPitch    
      // 0.0,                  // 10 neckYaw
      // 0.0,                  // 11 upperNeckPitch
    init_drake_body_joint_pos[4] = 0.30019663134302466;  // 12 rightShoulderPitch
    init_drake_body_joint_pos[5] = 1.25; // 13 rightShoulderRoll
    init_drake_body_joint_pos[6] = 0.0;  // 14 rightShoulderYaw
    init_drake_body_joint_pos[7] = 0.7853981633974483;   // 15 rightElbowPitch
    init_drake_body_joint_pos[8]  = 1.571;                // 16 rightForearmYaw
    init_drake_body_joint_pos[9]  =   0.0;                // 17 rightWristRoll
    init_drake_body_joint_pos[10] =   0.0;                // 18 rightWristPItch
    init_drake_body_joint_pos[11] = 0.30019663134302466;  // 19 leftShoulderPitch
    init_drake_body_joint_pos[12] = -1.25;                // 20 leftShoulderRoll
    init_drake_body_joint_pos[13] =   0.0;                // 22 leftShoulderYaw
    init_drake_body_joint_pos[14] = -0.7853981633974483;  // 22 leftElbowPitch
    init_drake_body_joint_pos[15] = 1.571;                // 23 leftForearmYaw
    init_drake_body_joint_pos[16] =   0.0;                // 24 leftWristRoll
    init_drake_body_joint_pos[17] =   0.0;                // 25 LeftWristPitch
    init_drake_body_joint_pos[18] =   0.0;                // 26 rightHipYaw
    init_drake_body_joint_pos[19] =   0.0;                // 27 rightHipRoll
    init_drake_body_joint_pos[20] = -0.49;                // 28 rightHipPitch
    init_drake_body_joint_pos[21] = 1.205;                // 29 rightKneePitch
    init_drake_body_joint_pos[22] = -0.71;                // 30 rightAnklePitch
    init_drake_body_joint_pos[23] =   0.0;                  // 31 rightAnkleRoll
    init_drake_body_joint_pos[24] =   0.0;                  // 32 leftHipYaw
    init_drake_body_joint_pos[25] =   0.0;                  // 33 leftHipRoll
    init_drake_body_joint_pos[26] = -0.49;                // 34 leftHipPitch
    init_drake_body_joint_pos[27] = 1.205;                // 35 leftKneePitch
    init_drake_body_joint_pos[28] = -0.71;                // 36 leftAnklePitch
    init_drake_body_joint_pos[29] =   0.0;                // 37 leftAnkleRoll

}

void update_floating_joint_pos(std::vector<float>   &init_drake_floating_joint_pos,
                               sensor_msgs::JointState &floating_joint_update){

    for (size_t i = 0; i < init_drake_floating_joint_pos.size(); i++){
        init_drake_floating_joint_pos[i] = floating_joint_update.position[i];

    }

}

void update_body_joint_pos(std::vector<float>   &init_drake_body_joint_pos,
                           sensor_msgs::JointState &body_joint_update){

    for (size_t i = 0; i < init_drake_body_joint_pos.size(); i++){
        init_drake_body_joint_pos[i] = body_joint_update.position[i];

    }

}

void define_desired_hand_pos(std::vector<val_ik_msgs::BodyPositionConstraint> &desired_body_positions, 
                             float x, float y, float z, bool offset_from_current){
    val_ik_msgs::BodyPositionConstraint body_constraint;

    body_constraint.body_name = "rightPalm";    
    body_constraint.offset_from_current = offset_from_current;
    body_constraint.world_position.x = x;
    body_constraint.world_position.y = y;
    body_constraint.world_position.z = z;


    desired_body_positions.push_back(body_constraint);
}

void print_input_joint_names(){
    std::vector<std::string>                                 drake_floating_joint_names;
    std::vector<std::string>                                 drake_body_joint_names;
    std::vector<float>                                       init_drake_body_joint_pos;
    std::vector<float>                                       init_drake_floating_joint_pos;
    std::vector<val_ik_msgs::BodyPositionConstraint>         desired_body_positions;
    std::vector<val_ik_msgs::BodyQuaternionConstraint>       desired_quaternion_positions;
    std::vector<val_ik_msgs::JointPositionConstraint>        desired_joint_positions;

}

void publish_robot_states(tf::TransformBroadcaster &br, 
                          const sensor_msgs::JointState &floating_joint_state_response,
                          const sensor_msgs::JointState &body_joint_state_response,
                          const ros::Publisher      &joint_state_pub, std::string frame_id){
    tf::Transform transform;
    tf::Quaternion q;
    float body_x = floating_joint_state_response.position[0]; 
    float body_y = floating_joint_state_response.position[1];
    float body_z = floating_joint_state_response.position[2];    

    float body_roll = floating_joint_state_response.position[3]; 
    float body_pitch = floating_joint_state_response.position[4];
    float body_yaw = floating_joint_state_response.position[5]; 

    transform.setOrigin( tf::Vector3(body_x, body_y, body_z) );

    // Rotation from fixed axis
    tf::Quaternion q_world_roll;   q_world_roll.setRPY(body_roll, 0.0, 0.0); 
    tf::Quaternion q_world_pitch;  q_world_pitch.setRPY(0.0, body_pitch, 0.0); 
    tf::Quaternion q_world_yaw;    q_world_yaw.setRPY(0.0, 0.0, body_yaw); 

    // Normalize Quaternions
    q_world_roll.normalize();     q_world_pitch.normalize();     q_world_yaw.normalize();

    // Extrinsict Rotation about Space-fixed x-y-z axes by R-P-Y angles respectively.
    // equivalently, we use an instrinsic rotation about body frames z-y`-x`` 
    // Convention of Drake as seen in math::rol_pitch_yaw_not_using_quaternions
    q = q_world_yaw*q_world_pitch*q_world_roll;

    //q.setRPY(body_roll , body_pitch , body_yaw);
    transform.setRotation(q);
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "my_origin", frame_id));
    joint_state_pub.publish(body_joint_state_response);
}

ros::ServiceClient  client;

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "test_val_ik_service_call");
    // Declare Node Handle
    ros::NodeHandle nh;
    
    // Declare Service Client 
    client = nh.serviceClient<val_ik::DrakeIKVal>("val_ik/val_ik_service");
    val_ik::DrakeIKVal ik_srv;

    ROS_INFO("Waiting for val_ik_service");
    ros::service::waitForService("val_ik/val_ik_service", -1);
    ROS_INFO("Service is ready");

    // Declare Joint State Pub
    ros::Publisher ik_joint_state_pub  = nh.advertise<sensor_msgs::JointState>( "/robot1/joint_states", 100 );  
    ros::Publisher robot_joint_state_pub  = nh.advertise<sensor_msgs::JointState>( "/robot2/joint_states", 100 );      


    // Prepare Client Message
    std::vector<std::string>                                 drake_floating_joint_names;
    std::vector<std::string>                                 drake_body_joint_names;
    std::vector<float>                                       init_drake_body_joint_pos;
    std::vector<float>                                       init_drake_floating_joint_pos;
    std::vector<val_ik_msgs::BodyPositionConstraint>         desired_body_positions;
    std::vector<val_ik_msgs::BodyQuaternionConstraint>       desired_quaternion_positions;
    std::vector<val_ik_msgs::JointPositionConstraint>        desired_joint_positions;

    // Initialize IK Joint Names
    init_IK_joint_names(drake_floating_joint_names, drake_body_joint_names);
    init_IK_positions(drake_floating_joint_names, drake_body_joint_names, init_drake_floating_joint_pos, init_drake_body_joint_pos);
    define_IK_init_positions_test(init_drake_floating_joint_pos, init_drake_body_joint_pos);
    //define_desired_hand_pos(desired_body_positions, 0.4, -0.2, 1.0, false);
    define_desired_hand_pos(desired_body_positions, 0.0, 0.0, 0.0, true);    

    ik_srv.request.drake_floating_joint_names = drake_floating_joint_names;
    ik_srv.request.drake_body_joint_names = drake_body_joint_names;    

    ik_srv.request.init_drake_body_joint_pos = init_drake_body_joint_pos;
    ik_srv.request.init_drake_floating_joint_pos = init_drake_floating_joint_pos;   

    ik_srv.request.desired_body_positions = desired_body_positions;
    ik_srv.request.desired_quaternion_positions = desired_quaternion_positions;        
    ik_srv.request.desired_joint_positions = desired_joint_positions;    


    // Initializing Transform Broacaster br
    tf::TransformBroadcaster br;

    // Preparing response field
    sensor_msgs::JointState floating_joint_state_response;
    sensor_msgs::JointState body_joint_state_response;

    std::vector<sensor_msgs::JointState>  floating_joint_state_res_vec;
    std::vector<sensor_msgs::JointState>  body_joint_state_res_vec;

    // register ctrl-c
    signal(SIGINT, sig_handler);    
    ros::Rate r(5);
    bool callOnce = false;
    float t = 0;
    float x_prev = 0.0;

    std_msgs::Header joint_header;
    joint_header.seq = 0;
    joint_header.stamp = ros::Time::now();

    float start_x = 0.0567334; float start_y = -0.423633; float start_z = 0.781989; 
    float desired_x = start_x;     float desired_y = start_y;     float desired_z = start_z;

    // Preparing response field
    sensor_msgs::JointState initial_floating_joint_state_response;
    sensor_msgs::JointState initial_body_joint_state_response;
    bool firstCall = false;

    while (!g_caught_sigint && ros::ok()){
        float x_now =  std::fabs(0.3*std::sin(t));
        float dx = x_now - x_prev;
        x_prev = x_now;
        std::cout << t << std::endl;
        std::cout << "x_now" << x_now <<  std::endl;
        std::cout << "dx" << dx <<  std::endl;          

        std::vector<val_ik_msgs::BodyPositionConstraint>         dx_desired_body_positions;

        desired_x += dx;
        desired_y += dx;
        desired_z += dx;                

        define_desired_hand_pos(dx_desired_body_positions, desired_x, desired_y, desired_z, false);    
        ik_srv.request.desired_body_positions = dx_desired_body_positions;

        // Begin Service Call
        if (client.call(ik_srv)){
            ROS_INFO("Call Successful");
            floating_joint_state_response = ik_srv.response.robot_joint_states.floating_joint_states;
            body_joint_state_response = ik_srv.response.robot_joint_states.body_joint_states;

            std::cout << "In Floating Joints Names Size:" <<  ik_srv.response.robot_joint_states.floating_joint_states.name.size() << std::endl;
            std::cout << "Out Floating Joints Names Size:" << floating_joint_state_response.name.size() << std::endl;            

            std::cout << "In Body Joints Names Size:" <<  ik_srv.response.robot_joint_states.body_joint_states.name.size() << std::endl;
            std::cout << "Out Body Joints Names Size:" << body_joint_state_response.name.size() << std::endl;

            update_floating_joint_pos(init_drake_floating_joint_pos, floating_joint_state_response);
            update_body_joint_pos(init_drake_body_joint_pos,     body_joint_state_response);


            ik_srv.request.init_drake_floating_joint_pos = init_drake_floating_joint_pos;
            ik_srv.request.init_drake_body_joint_pos = init_drake_body_joint_pos;            

            joint_header.seq += 1;
            joint_header.stamp = ros::Time::now();

            initial_floating_joint_state_response.header = joint_header;
            initial_body_joint_state_response.header = joint_header;

            body_joint_state_response.header = joint_header;
            floating_joint_state_response.header = joint_header;

            floating_joint_state_res_vec.push_back(floating_joint_state_response);
            body_joint_state_res_vec.push_back(body_joint_state_response);



            if (!firstCall){
                initial_floating_joint_state_response = ik_srv.response.robot_joint_states.floating_joint_states;
                initial_body_joint_state_response =  ik_srv.response.robot_joint_states.body_joint_states;
                firstCall = true;
            }

            publish_robot_states(br, floating_joint_state_response, body_joint_state_response, ik_joint_state_pub, "val_ik_robot/pelvis");



        }
        else{
           ROS_ERROR("Failed to call service val_ik/val_ik_service");
        }

        if (firstCall){
            publish_robot_states(br, initial_floating_joint_state_response, initial_body_joint_state_response, robot_joint_state_pub, "val_robot/pelvis");           
        }

        ros::spinOnce();
        t = t + 0.2;
        r.sleep();
        if (t > 3.1){
            break;
        }

    }
    int num_stored = body_joint_state_res_vec.size();
    int counter = 0;    
    while (!g_caught_sigint && ros::ok()){
        publish_robot_states(br, initial_floating_joint_state_response, initial_body_joint_state_response, robot_joint_state_pub, "val_robot/pelvis");           
        publish_robot_states(br, floating_joint_state_res_vec[counter], body_joint_state_res_vec[counter], ik_joint_state_pub,  "val_ik_robot/pelvis");
        counter += 1;
        counter = counter % num_stored;


        ros::spinOnce();
        r.sleep();
    }    
    return 0;
}
    