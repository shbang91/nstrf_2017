#include "val_ik/global_vars.h"

namespace val_ik_global{
    const std::vector<std::string> drake_robot_state_names = {
        "base_x",
        "base_y", 
        "base_z",
        "base_roll", 
        "base_pitch", 
        "base_yaw",
        "torsoYaw",
        "torsoPitch",
        "torsoRoll",
        "lowerNeckPitch",    
        "rightShoulderPitch",
        "rightShoulderRoll",
        "rightShoulderYaw",
        "rightElbowPitch",
        "rightForearmYaw",
        "rightWristRoll",
        "rightWristPitch",
        "leftShoulderPitch",
        "leftShoulderRoll",
        "leftShoulderYaw",
        "leftElbowPitch",
        "leftForearmYaw",
        "leftWristRoll",
        "leftWristPitch",
        "rightHipYaw",
        "rightHipRoll",
        "rightHipPitch",
        "rightKneePitch",
        "rightAnklePitch",
        "rightAnkleRoll",
        "leftHipYaw",
        "leftHipRoll",
        "leftHipPitch",
        "leftKneePitch",
        "leftAnklePitch",
        "leftAnkleRoll"  
    };

    const std::map<std::string, int> drake_state_name_to_state_index = {
        {"base_x", 0},
        {"base_y", 1},
        {"base_z", 2},
        {"base_roll", 3} ,
        {"base_pitch", 4},
        {"base_yaw", 5},
        {"torsoYaw", 6},
        {"torsoPitch", 7},
        {"torsoRoll", 8},
        {"lowerNeckPitch", 9},
        {"rightShoulderPitch", 10},
        {"rightShoulderRoll", 11},
        {"rightShoulderYaw", 12},
        {"rightElbowPitch", 13},
        {"rightForearmYaw", 14},
        {"rightWristRoll", 15},
        {"rightWristPitch", 16},
        {"leftShoulderPitch", 17},
        {"leftShoulderRoll", 18},
        {"leftShoulderYaw", 19},
        {"leftElbowPitch", 20},
        {"leftForearmYaw", 21},
        {"leftWristRoll", 22},
        {"leftWristPitch", 23},
        {"rightHipYaw", 24},
        {"rightHipRoll", 25},
        {"rightHipPitch", 26},
        {"rightKneePitch", 27},
        {"rightAnklePitch", 28},
        {"rightAnkleRoll", 29},
        {"leftHipYaw", 30},
        {"leftHipRoll", 31},
        {"leftHipPitch", 32},
        {"leftKneePitch", 33},
        {"leftAnklePitch", 34},
        {"leftAnkleRoll", 35}
    };

    const std::map<int, std::string> drake_state_index_to_state_name = {
        {0, "base_x"},
        {1, "base_y"},
        {2, "base_z"},
        {3, "base_roll"} ,
        {4, "base_pitch"},
        {5, "base_yaw"},
        {6, "torsoYaw"},
        {7, "torsoPitch"},
        {8, "torsoRoll"},
        {9, "lowerNeckPitch"},
        {10, "rightShoulderPitch"},
        {11, "rightShoulderRoll"},
        {12, "rightShoulderYaw"},
        {13, "rightElbowPitch"},
        {14, "rightForearmYaw"},
        {15, "rightWristRoll"},
        {16, "rightWristPitch"},
        {17, "leftShoulderPitch"},
        {18, "leftShoulderRoll"},
        {19, "leftShoulderYaw"},
        {20, "leftElbowPitch"},
        {21, "leftForearmYaw"},
        {22, "leftWristRoll"},
        {23, "leftWristPitch"},
        {24, "rightHipYaw"},
        {25, "rightHipRoll"},
        {26, "rightHipPitch"},
        {27, "rightKneePitch"},
        {28, "rightAnklePitch"},
        {29, "rightAnkleRoll"},
        {30, "leftHipYaw"},
        {31, "leftHipRoll"},
        {32, "leftHipPitch"},
        {33, "leftKneePitch"},
        {34, "leftAnklePitch"},
        {35, "leftAnkleRoll"}
    };
    
    void output_global_vars(){
        for(int i = 0; i < drake_robot_state_names.size(); i++){
            std::string robot_state_name = drake_robot_state_names.at(i);
            std::string robot_state_name_map_test = drake_state_index_to_state_name.at(i);
            int index_map_test = drake_state_name_to_state_index.at(robot_state_name_map_test);          
             std::cout << i << " " << index_map_test << " "
                                   << robot_state_name << " " 
                                   << robot_state_name_map_test << std::endl;

        }
    }



}
