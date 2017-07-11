#include <ros/ros.h>

#include <signal.h>

// Include drake dependencies to perform IK
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/multibody/rigid_body_plant/viewer_draw_translator.h"
#include "drake/multibody/rigid_body_tree.h"

// Include ROS message types
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "val_ik_msgs/BodyPositionConstraint.h"
#include "val_ik_msgs/BodyQuaternionConstraint.h"
#include "val_ik_msgs/JointPositionConstraint.h"
#include "val_ik_msgs/RobotJointStates.h"

// Include ROS Service
#include "val_ik/DrakeIKVal.h"
#include "val_ik/DrakeFKBodyPose.h"

#include <tf/transform_broadcaster.h>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;

namespace drake{
    void send_lcm(const RigidBodyTreed* tree, VectorXd q_sol){
      VectorX<double> x = VectorX<double>::Zero(tree->get_num_positions() +
                                                tree->get_num_velocities());
      x.head(q_sol.size()) = q_sol;
      systems::BasicVector<double> q_draw(x);        
        lcm::DrakeLcm lcm;
        std::vector<uint8_t> message_bytes;

        lcmt_viewer_load_robot load_msg =
        multibody::CreateLoadRobotMessage<double>(*tree);
        const int length = load_msg.getEncodedSize();
        message_bytes.resize(length);
        load_msg.encode(message_bytes.data(), 0, length);
        lcm.Publish("DRAKE_VIEWER_LOAD_ROBOT", message_bytes.data(),
        message_bytes.size());

        systems::ViewerDrawTranslator posture_drawer(*tree);
        posture_drawer.Serialize(0, q_draw, &message_bytes);
        lcm.Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
        message_bytes.size());
    }
}

using namespace drake;

sensor_msgs::JointState joint_state_msg;
ros::Publisher          joint_state_pub;
ros::ServiceServer      val_ik_srv;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

// Initialize Global Tree Value
auto tree = std::make_unique<RigidBodyTree<double>>();


/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


/* Finds and returns the indices within the state vector of @p tree that contain
 * the position states of a joint named @p name. The model instance ID is
 * ignored in this search (joints belonging to all model instances are
 * searched).
 */
std::vector<int> GetJointPositionVectorIndices(const RigidBodyTreed* tree,
                                               const std::string& name) {
  RigidBody<double>* joint_child_body = tree->FindChildBodyOfJoint(name);
  int num_positions = joint_child_body->getJoint().get_num_positions();
  std::vector<int> ret(static_cast<size_t>(num_positions));

  // Since the joint position states are located in a contiguous region of the
  // the rigid body tree's state vector, fill the return vector with
  // sequentially increasing indices starting at
  // `joint_child_body->get_position_start_index()`.
  std::iota(ret.begin(), ret.end(), joint_child_body->get_position_start_index
  ());
  return ret;
}

/* Finds the indices within the state vector of @p tree that contains the
 * position states of a joint named @p name, and appends the vector of
 * indices found to the end of @p position_list.
 */
void FindJointAndInsert(const RigidBodyTreed* model, const std::string& name,
                        std::vector<int>* const position_list) {
  auto position_indices = GetJointPositionVectorIndices(model, name);

  position_list->insert(position_list->end(), position_indices.begin(),
                        position_indices.end());
}

void init_IK_global_vars(){
  // Initialize Tree and URDF
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      multibody::joints::kRollPitchYaw, tree.get());

}

bool FKServiceCallback(val_ik::DrakeFKBodyPose::Request& req, val_ik::DrakeFKBodyPose::Response& res){
    std::vector<geometry_msgs::Pose>  body_poses;

    // define reach_start
    //KinematicsCache<double> cache = tree->doKinematics(reach_start);

    for(size_t i = 0; i < req.body_names.size(); i++){
        int body_index = tree->FindBodyIndex(req.body_names[i]);
       // auto body_pose = tree->relativeTransform(cache, 0, body_index);
        // Get Position and Quaternion
          //  const auto& body_xyz = body_pose.translation();
         //   Vector4d body_quat = drake::math::rotmat2quat(body_pose.linear());        

        // Populate msg
        geometry_msgs::Pose this_body_pose;
        //this_body_pose.position.x = ;
        // this_body_pose.orientation.x =;
        body_poses.push_back(this_body_pose);
        //
    }

    return true;

}


bool ikServiceCallback(val_ik::DrakeIKVal::Request& req, val_ik::DrakeIKVal::Response& res){
    ROS_INFO("ikServiceCallback: processing ik request");

    std::cout << "world:" << tree->get_body(0).get_name() << std::endl;
    std::cout << "root_body:" << tree->get_body(1).get_name() << std::endl;
    std::cout << "root_joint:" <<  tree->get_body(1).getJoint().get_name() << std::endl;    

    std::cout << "drake_floating_joint_names size: " << req.drake_floating_joint_names.size() << std::endl;
    std::cout << "init_drake_floating_joint_pos size: " << req.init_drake_floating_joint_pos.size() << std::endl;

    std::cout << "drake_body_joint_names size: " << req.drake_body_joint_names.size() << std::endl;
    std::cout << "init_drake_body_joint_pos size: " << req.init_drake_body_joint_pos.size() << std::endl;  

    // Store Body Names which has a requested world position constraint
    std::vector<std::string> request_constrained_body_positions;
    // Create map of body name to index
    std::map<std::string, int> position_constrained_body_to_index;
    for (size_t i = 0; i < req.desired_body_positions.size(); i++){
        std::string constrained_body_name; 
        constrained_body_name = req.desired_body_positions[i].body_name;
        request_constrained_body_positions.push_back(constrained_body_name);
        position_constrained_body_to_index[constrained_body_name] = i;
    }

    // Initialize generalized coordinate positions
    size_t num_floating_joints = req.init_drake_floating_joint_pos.size();
    size_t num_body_joints = req.init_drake_body_joint_pos.size();  
    VectorXd reach_start(tree->get_num_positions());
    for (size_t i = 0; i < ( num_floating_joints + num_body_joints); i++){
        if (i < num_floating_joints){
            reach_start[i] = req.init_drake_floating_joint_pos[i];
        }else{
            size_t j = i - num_floating_joints ;
            reach_start[i] = req.init_drake_body_joint_pos[j];
        }
    }

    // Debug input
    std::cout << "reach_start size:" << reach_start.size() << std::endl;   
    for (size_t i = 0; i < reach_start.size(); i++){
        std::cout << reach_start[i] << std::endl;
    }


    double inf = std::numeric_limits<double>::infinity();
    Vector2d tspan;
    tspan << 0, inf;

    KinematicsCache<double> cache = tree->doKinematics(reach_start);

    // 1 Neck Posture Constraint, posture constraints are imposed on q
    PostureConstraint kc_posture_neck(tree.get(), tspan);
    std::vector<int> neck_idx;
    FindJointAndInsert(tree.get(), "lowerNeckPitch", &neck_idx);
    FindJointAndInsert(tree.get(), "neckYaw", &neck_idx);
    FindJointAndInsert(tree.get(), "upperNeckPitch", &neck_idx);
    VectorXd neck_lb = VectorXd::Zero(neck_idx.size());
    VectorXd neck_ub = VectorXd::Zero(neck_idx.size());
    kc_posture_neck.setJointLimits(neck_idx.size(), neck_idx.data(), neck_lb,
                                   neck_ub);

    // 2 Left foot position and orientation constraint, position and orientation
    // constraints are imposed on frames/bodies
    const Vector3d origin(0, 0, 0);

    int l_foot = tree->FindBodyIndex("leftFoot");
    Vector4d lfoot_quat(1, 0, 0, 0);                // This should be changed to the current foot position
    auto lfoot_pos0 = tree->transformPoints(cache, origin, l_foot, 0);
    Vector3d lfoot_pos_lb = lfoot_pos0;
    // Position and quaternion constraints are relaxed to make the problem
    // solvable by IPOPT.
    lfoot_pos_lb -= 0.0001*Vector3d::Ones();
    Vector3d lfoot_pos_ub = lfoot_pos0;
    lfoot_pos_ub += 0.0001*Vector3d::Ones();
    WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, origin, lfoot_pos_lb,
                                         lfoot_pos_ub, tspan);
    double tol = 0.5 / 180 * M_PI;
    WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol, tspan);

    // 3 Right foot position and orientation constraint
    int r_foot = tree->FindBodyIndex("rightFoot");
    auto rfoot_pos0 = tree->transformPoints(cache, origin, r_foot, 0);
    Vector4d rfoot_quat(1, 0, 0, 0);
    Vector3d rfoot_pos_lb = rfoot_pos0;
    rfoot_pos_lb -= 0.0001*Vector3d::Ones();
    Vector3d rfoot_pos_ub = rfoot_pos0;
    rfoot_pos_ub += 0.0001*Vector3d::Ones();

    WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, origin, rfoot_pos_lb,
                                         rfoot_pos_ub, tspan);
    WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol, tspan);

    // 4 Torso posture constraint
    PostureConstraint kc_posture_torso(tree.get(), tspan);
    std::vector<int> torso_idx;
    FindJointAndInsert(tree.get(), "torsoYaw", &torso_idx);
    FindJointAndInsert(tree.get(), "torsoPitch", &torso_idx);
    FindJointAndInsert(tree.get(), "torsoRoll", &torso_idx);
    Vector3d torso_nominal = Vector3d::Zero();
    Vector3d torso_half_range(11.0 / 180 * M_PI, 15.0 / 180 * M_PI, 1.0 / 180 * M_PI);
//    Vector3d torso_half_range(1.0 / 180 * M_PI, 1.0 / 180 * M_PI, 1.0 / 180 * M_PI);  
    Vector3d torso_lb = torso_nominal - torso_half_range;
    Vector3d torso_ub = torso_nominal + torso_half_range;
    torso_lb(1) = -5.0 / 180 * M_PI;
    kc_posture_torso.setJointLimits(3, torso_idx.data(), torso_lb, torso_ub);

    // 5 knee posture constraint
    PostureConstraint kc_posture_knee(tree.get(), tspan);
    std::vector<int> knee_idx;
    FindJointAndInsert(tree.get(), "leftKneePitch", &knee_idx);
    FindJointAndInsert(tree.get(), "rightKneePitch", &knee_idx);
    Vector2d knee_lb(-4.0/180*M_PI, -4.0/180*M_PI);
    Vector2d knee_ub(115.0/180*M_PI, 115.0/180*M_PI);
    kc_posture_knee.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

    // 6 Left arm posture constraint
    PostureConstraint kc_posture_larm(tree.get(), tspan);
    std::vector<int> larm_idx;
    FindJointAndInsert(tree.get(), "leftShoulderPitch", &larm_idx);
    FindJointAndInsert(tree.get(), "leftShoulderRoll", &larm_idx);
    FindJointAndInsert(tree.get(), "leftShoulderYaw", &larm_idx);
    FindJointAndInsert(tree.get(), "leftElbowPitch", &larm_idx);
    FindJointAndInsert(tree.get(), "leftForearmYaw", &larm_idx);
    FindJointAndInsert(tree.get(), "leftWristRoll", &larm_idx);
    FindJointAndInsert(tree.get(), "leftWristPitch", &larm_idx);
    Eigen::Matrix<double, 7, 1> larm_lb;
    larm_lb.setZero();
    for (int i = 0; i < 7; i++) larm_lb(i) = reach_start(larm_idx[i]);
    Eigen::Matrix<double, 7, 1> larm_ub = larm_lb;
    kc_posture_larm.setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);

    // 7 Right arm posture constraint
    PostureConstraint kc_posture_rarm(tree.get(), tspan);
    std::vector<int> rarm_idx;
    FindJointAndInsert(tree.get(), "rightShoulderPitch", &rarm_idx);
    FindJointAndInsert(tree.get(), "rightShoulderRoll", &rarm_idx);
    FindJointAndInsert(tree.get(), "rightShoulderYaw", &rarm_idx);
    FindJointAndInsert(tree.get(), "rightElbowPitch", &rarm_idx);
    FindJointAndInsert(tree.get(), "rightForearmYaw", &rarm_idx);
    FindJointAndInsert(tree.get(), "rightWristRoll", &rarm_idx);
    FindJointAndInsert(tree.get(), "rightWristPitch", &rarm_idx);
    Eigen::Matrix<double, 7, 1> rarm_lb;
    rarm_lb.setZero();
    for (int i = 0; i < 7; i++) rarm_lb(i) = reach_start(rarm_idx[i]);
    Eigen::Matrix<double, 7, 1> rarm_ub = rarm_lb;
    kc_posture_rarm.setJointLimits(7, rarm_idx.data(), rarm_lb, rarm_ub);


    // 2 Left foot position and orientation constraint, position and orientation
    // constraints are imposed on frames/bodies
    const Vector3d world_origin(0, 0, 0);

    int rh_palm = tree->FindBodyIndex("rightPalm");
    Vector4d rh_palm_quat(0.707, 0, 0, 0.707);
    auto rh_palm0 = tree->transformPoints(cache, world_origin, rh_palm, 0);

    std::cout << "rh_palm0:" << rh_palm0[0] << " " << rh_palm0[1] << " " << rh_palm0[2] << " " << std::endl;


  //  double tol = 0.5 / 180 * M_PI;
    WorldQuatConstraint kc_rh_palm_quat(tree.get(), rh_palm, rh_palm_quat, tol, tspan);

    // Position and quaternion constraints are relaxed to make the problem
    // solvable by IPOPT.
    Vector3d rh_palm_des_offset(0.0, 0.0, 0.0);  

    if(std::find(request_constrained_body_positions.begin(), 
                 request_constrained_body_positions.end(), "rightPalm") != request_constrained_body_positions.end()) {
        ROS_INFO("rightPalm Position Request");
        geometry_msgs::Point world_point;
        int index = position_constrained_body_to_index["rightPalm"];
        world_point = req.desired_body_positions[index].world_position;
        ROS_INFO("x,y,z");
        std::cout << world_point.x << " " << world_point.y << " " << world_point.z << " " << std::endl;

        Vector3d rpalm_desired_world_pos( world_point.x,  world_point.y,  world_point.z);

        rh_palm_des_offset = rpalm_desired_world_pos;
        if (!req.desired_body_positions[index].offset_from_current){
            rh_palm_des_offset = rpalm_desired_world_pos - rh_palm0;
        }

        std::cout << rh_palm_des_offset[0] << " " << rh_palm_des_offset[1] << " " << rh_palm_des_offset[2] << " " << std::endl;        
    }


    Vector3d pos_end;
    pos_end = rh_palm0 + rh_palm_des_offset;

    ROS_INFO("Pos end:");
    std::cout << pos_end[0] << " " << pos_end[1] << " " << pos_end[2] << " " << std::endl;        

    const double pos_tol = 0.01;
    const Vector3d pos_lb = pos_end - Vector3d::Constant(pos_tol);
    const Vector3d pos_ub = pos_end + Vector3d::Constant(pos_tol);

    WorldPositionConstraint kc_rh_palm_pos(tree.get(), rh_palm, world_origin, pos_lb, pos_ub, tspan);


    int pelvis = tree->FindBodyIndex("pelvis");
//    auto pelvis0 = tree->transformPoints(cache, world_origin, pelvis0, 0);  
    Vector3d pelvis_pos_end(0,0, 1.025);
//    pelvis_pos_end = pelvis0;      
    Vector3d pelvis_pos_lb = pelvis_pos_end - Vector3d::Constant(pos_tol);
    Vector3d pelvis_pos_ub = pelvis_pos_end + Vector3d::Constant(pos_tol);    

    pelvis_pos_lb(2) = 1.05;
    pelvis_pos_lb(2) = 0.9;    

    Vector4d pelvis_quat(1, 0, 0, 0);
  //  double tol = 0.5 / 180 * M_PI;
    WorldQuatConstraint kc_pelvis_quat(tree.get(), pelvis, pelvis_quat, tol, tspan);

    WorldPositionConstraint kc_pelvis_pos(tree.get(), pelvis, world_origin, pelvis_pos_lb, pelvis_pos_ub, tspan);

    // 8 Quasistatic constraint
    QuasiStaticConstraint kc_quasi(tree.get(), tspan);
    kc_quasi.setShrinkFactor(0.4);
    kc_quasi.setActive(true);

    auto leftFootPtr = tree->FindBody("leftFoot");
    Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
    Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
    kc_quasi.addContact(1, &l_foot, &l_foot_pts);

    auto rightFootPtr = tree->FindBody("rightFoot");
    Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
    Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
    kc_quasi.addContact(1, &r_foot, &r_foot_pts);

    // -----------------solve-----------------------------------------------------
    std::vector<RigidBodyConstraint*> constraint_array;
    constraint_array.push_back(&kc_posture_neck);
    constraint_array.push_back(&kc_lfoot_pos);
    constraint_array.push_back(&kc_lfoot_quat);
    constraint_array.push_back(&kc_rfoot_pos);
    constraint_array.push_back(&kc_rfoot_quat);
    constraint_array.push_back(&kc_posture_torso);
    constraint_array.push_back(&kc_posture_knee);
    constraint_array.push_back(&kc_posture_larm);
  //  constraint_array.push_back(&kc_posture_rarm);
    constraint_array.push_back(&kc_quasi);

    constraint_array.push_back(&kc_rh_palm_pos);
  //  constraint_array.push_back(&kc_rh_palm_quat);  
    constraint_array.push_back(&kc_pelvis_pos);
    constraint_array.push_back(&kc_pelvis_quat);

    

    IKoptions ikoptions(tree.get());
    VectorXd q_sol(tree->get_num_positions());
    VectorXd q_nom = reach_start;
    int info;
    std::vector<std::string> infeasible_constraint;
    std::cout << "start IK" << std::endl;
    inverseKin(tree.get(), q_nom, q_nom, constraint_array.size(),
               constraint_array.data(), ikoptions, &q_sol, &info,
               &infeasible_constraint);

    std::cout << "done!" << std::endl;

    // After solving
    Vector3d com = tree->centerOfMass(cache);



    // Return IK Solution:

    // Prepare Joint State Message Response
    sensor_msgs::JointState floating_joint_state_msg;
    for(size_t i = 0; i < req.drake_floating_joint_names.size(); i++ ){
      std::string joint_name;
      joint_name = req.drake_floating_joint_names[i];
      double joint_pos = q_sol[i];   
      floating_joint_state_msg.name.push_back(joint_name);
      floating_joint_state_msg.position.push_back(joint_pos);
    }


    // Prepare Joint State Message Response
    sensor_msgs::JointState body_joint_state_msg;
    for(size_t i = 0; i < req.drake_body_joint_names.size(); i++ ){
      std::string joint_name;
      joint_name = req.drake_body_joint_names[i];
      int joint_id = GetJointPositionVectorIndices(tree.get(), joint_name)[0];
      double joint_pos = q_sol[joint_id];   
      body_joint_state_msg.name.push_back(joint_name);
      body_joint_state_msg.position.push_back(joint_pos);
    }

    // Debug Joint Inputs
    // Return Same Body Joint Positions
/*    for (size_t i = 0; i < num_body_joints; i++){
        body_joint_state_msg.name.push_back(req.drake_body_joint_names[i]);
        body_joint_state_msg.position.push_back(req.init_drake_body_joint_pos[i]);        
    }*/

    ROS_INFO("Printing q_sol:");
    for (size_t i = 0; i < q_sol.size(); i++)
{        std::cout << "    " << "i:" << i << " value:" << q_sol[i] << std::endl;  
    }


    res.robot_joint_states.floating_joint_states = floating_joint_state_msg;    
    res.robot_joint_states.body_joint_states = body_joint_state_msg;

    ROS_INFO("    Request ended successfully. returning true");


//    drake::send_lcm(tree.get(), q_sol);
//    joint_state_pub.publish(body_joint_state_msg);

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "val_ik");
  ros::NodeHandle       node;

  //joint_state_pub = node.advertise<sensor_msgs::JointState>( "/robot1/joint_states", 0 );
  val_ik_srv = node.advertiseService("val_ik/val_ik_service", ikServiceCallback);
  val_ik_srv = node.advertiseService("val_ik/val_fk_service", FKServiceCallback);  

  // Initialize Service Global Variables
  init_IK_global_vars();

  // register ctrl-c
  signal(SIGINT, sig_handler);

  double ros_rate = 30.0;
  ros::Rate r(ros_rate);

  ROS_INFO("val_ik/val_ik_service is ready");
  ROS_INFO("    Waiting for service calls");
  // Main loop:
  while (!g_caught_sigint && ros::ok()){
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
