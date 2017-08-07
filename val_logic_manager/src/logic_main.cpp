#include "logic_main.h"
// Declare Global Objects
bool global_state_update_received = false;
bool global_first_state_update_received = false;
boost::mutex state_mutex;
sensor_msgs::JointState      global_joint_state_msg;
nav_msgs::Odometry           global_odom_msg;


int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "val_logic_manager");
    LogicManager logic_manager;

    // Declare Service Call clients
    // IK Client
    logic_manager.ik_manager.ik_client        = logic_manager.nh.serviceClient<val_ik::DrakeIKVal>("val_ik/val_ik_service");
    logic_manager.ik_manager.single_ik_client = logic_manager.nh.serviceClient<val_ik::DrakeOneHandSingleIk>("val_ik/val_single_ik_service");
    logic_manager.ik_manager.fk_client  = logic_manager.nh.serviceClient<val_ik::DrakeFKBodyPose>("val_ik/val_fk_service");

    ROS_INFO("Waiting for package val_ik services ");
    ros::service::waitForService("val_ik/val_ik_service", -1);
    ros::service::waitForService("val_ik/val_single_ik_service", -1);
    ros::service::waitForService("val_ik/val_fk_service", -1);        
    ROS_INFO("Services are ready");

    // Declare Subscribers
    logic_manager.interactive_marker_sub = logic_manager.nh.subscribe<visualization_msgs::InteractiveMarkerInit>("/basic_controls/update_full", 1, 
                                                                                                                  boost::bind(&LogicManager::interactive_callback, &logic_manager, _1));
    logic_manager.operator_command_sub = logic_manager.nh.subscribe<std_msgs::String>("val_logic_manager/operator_command", 1,
                                                                                       boost::bind(&LogicManager::operator_command_callback, &logic_manager, _1));

    logic_manager.grasploc_sub = logic_manager.nh.subscribe<valkyrie::GraspHandPoses>("/drake_ik_hand_trajectory", 1,
                                                                                       boost::bind(&LogicManager::grasploc_callback, &logic_manager, _1)); //grasploc
    // Synchronize Robot Joint State and Robot Pose
    message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub(logic_manager.nh, "/ihmc_ros/valkyrie/output/joint_states", 1);
    message_filters::Subscriber<nav_msgs::Odometry> robot_pose_sub(logic_manager.nh, "/ihmc_ros/valkyrie/output/robot_pose", 1);

    // ApproximateTime takes a queue size as its constructor argument, hence JointOdomSyncPolicy(10)
    message_filters::Synchronizer<JointOdomSyncPolicy> sync(JointOdomSyncPolicy(10), joint_state_sub, robot_pose_sub);
    sync.registerCallback(boost::bind(&stateFiltersCallback, _1, _2));

    // Declare Publishers
    logic_manager.val_ik_finalpose_robot_joint_states_pub = logic_manager.nh.advertise<sensor_msgs::JointState>("/val_ik_finalpose_robot/joint_states", 10);
    logic_manager.val_ik_initpose_robot_joint_states_pub = logic_manager.nh.advertise<sensor_msgs::JointState>("/val_ik_initpose_robot/joint_states", 10);
    logic_manager.marker_pub = logic_manager.nh.advertise<visualization_msgs::Marker>("val_logic_manager/sample_marker", 0);
    logic_manager.ihmc_wholebody_pub = logic_manager.nh.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 0);

    logic_manager.ihmc_go_home_pub = logic_manager.nh.advertise<ihmc_msgs::GoHomeRosMessage>("/ihmc_ros/valkyrie/control/go_home", 10);    

    logic_manager.ihmc_pelvis_traj_pub = logic_manager.nh.advertise<ihmc_msgs::PelvisTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/pelvis_trajectory", 10);    


    ros::Rate r(20);

    // Spin Forever
    while (ros::ok()){
        logic_manager.loop();
        r.sleep();
        ros::spinOnce(); 

    }

    return 0;
}