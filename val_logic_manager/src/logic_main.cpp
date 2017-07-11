#include "logic_main.h"

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "val_logic_manager");

    // Declare Node Handle
    ros::NodeHandle nh;

    // Declare Subscribers
    // Synchronize Point Cloud and Image Subscription Received Messages
    message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub(nh, "/ihmc_ros/valkyrie/output/joint_states", 1);
    message_filters::Subscriber<nav_msgs::Odometry> robot_pose_sub(nh, "/ihmc_ros/valkyrie/output/robot_pose", 1);

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), joint_state_sub, robot_pose_sub);
    sync.registerCallback(boost::bind(&stateFiltersCallback, _1, _2));

    ros::Rate r(20);

    // Spin Forever
    while (ros::ok()){
        ROS_INFO("Hello World");

        r.sleep();
        ros::spinOnce();        
    }

    return 0;
}