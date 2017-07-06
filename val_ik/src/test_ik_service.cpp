// This code tests the openpose rosservice node by loading an image and calling the service.#include <ros/ros.h>
#include <ros/ros.h>
#include <signal.h>

// Include ROS message types
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "val_ik_msgs/BodyPositionConstraint.h"
#include "val_ik_msgs/BodyQuaternionConstraint.h"
#include "val_ik_msgs/JointPositionConstraint.h"

// Include ROS Service
#include "val_ik/DrakeIKVal.h"

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
    drake_floating_joint_names.push_back("x");  
    drake_floating_joint_names.push_back("y"); 
    drake_floating_joint_names.push_back("z");
    drake_floating_joint_names.push_back("r"); 
    drake_floating_joint_names.push_back("p"); 
    drake_floating_joint_names.push_back("y");

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

ros::ServiceClient  client;
ros::Publisher      joint_state_pub;
int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "test_val_ik_service_call");
    // Declare Node Handle
    ros::NodeHandle nh;
    
    // Declare Service Client 
    client = nh.serviceClient<val_ik::DrakeIKVal>("val_ik/val_ik_service");
    val_ik::DrakeIKVal ik_srv;

    // Declare Joint State Pub
    joint_state_pub  = nh.advertise<sensor_msgs::JointState>( "/robot1/joint_states", 0 );  


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

    ik_srv.request.drake_floating_joint_names = drake_floating_joint_names;
    ik_srv.request.drake_body_joint_names = drake_body_joint_names;    

    ik_srv.request.init_drake_body_joint_pos = init_drake_body_joint_pos;
    ik_srv.request.init_drake_floating_joint_pos = init_drake_floating_joint_pos;   

    ik_srv.request.desired_body_positions = desired_body_positions;
    ik_srv.request.desired_quaternion_positions = desired_quaternion_positions;        
    ik_srv.request.desired_joint_positions = desired_joint_positions;    


    // Preparing response field
    sensor_msgs::JointState joint_state_response;

    // register ctrl-c
    signal(SIGINT, sig_handler);    

    bool callOnce = false;
    while (!g_caught_sigint && ros::ok()){
        // Begin Service Call
        if (!callOnce){
            if (client.call(ik_srv)){
                ROS_INFO("Call Successful");
                joint_state_response = ik_srv.response.robot_joint_states;
                std::cout << ik_srv.response.robot_joint_states.name.size() << std::endl;
                std::cout << joint_state_response.name.size() << std::endl;
                callOnce = true;
            }
            else{
               ROS_ERROR("Failed to call service val_ik/val_ik_service");
            }
        }
        if (callOnce){
            joint_state_pub.publish(joint_state_response);
        }
        ros::spinOnce();
    }
    
    return 0;
}
    