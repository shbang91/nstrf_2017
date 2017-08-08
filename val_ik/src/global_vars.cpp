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
        "neckYaw",        // 0.0,                  // 10 neckYaw
        "upperNeckPitch", // 0.0,                  // 11 upperNeckPitch        
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
        {"neckYaw", 10},
        {"upperNeckPitch", 11},                
        {"rightShoulderPitch", 12},
        {"rightShoulderRoll", 13},
        {"rightShoulderYaw", 14},
        {"rightElbowPitch", 15},
        {"rightForearmYaw", 16},
        {"rightWristRoll", 17},
        {"rightWristPitch", 18},
        {"leftShoulderPitch", 19},
        {"leftShoulderRoll", 20},
        {"leftShoulderYaw", 21},
        {"leftElbowPitch", 22},
        {"leftForearmYaw", 23},
        {"leftWristRoll", 24},
        {"leftWristPitch", 25},
        {"rightHipYaw", 26},
        {"rightHipRoll", 27},
        {"rightHipPitch", 28},
        {"rightKneePitch", 29},
        {"rightAnklePitch", 30},
        {"rightAnkleRoll", 31},
        {"leftHipYaw", 32},
        {"leftHipRoll", 33},
        {"leftHipPitch", 34},
        {"leftKneePitch", 35},
        {"leftAnklePitch", 36},
        {"leftAnkleRoll", 37}
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
        {10, "neckYaw"},
        {11, "upperNeckPitch"},                        
        {12, "rightShoulderPitch"},
        {13, "rightShoulderRoll"},
        {14, "rightShoulderYaw"},
        {15, "rightElbowPitch"},
        {16, "rightForearmYaw"},
        {17, "rightWristRoll"},
        {18, "rightWristPitch"},
        {19, "leftShoulderPitch"},
        {20, "leftShoulderRoll"},
        {21, "leftShoulderYaw"},
        {22, "leftElbowPitch"},
        {23, "leftForearmYaw"},
        {24, "leftWristRoll"},
        {25, "leftWristPitch"},
        {26, "rightHipYaw"},
        {27, "rightHipRoll"},
        {28, "rightHipPitch"},
        {29, "rightKneePitch"},
        {30, "rightAnklePitch"},
        {31, "rightAnkleRoll"},
        {32, "leftHipYaw"},
        {33, "leftHipRoll"},
        {34, "leftHipPitch"},
        {35, "leftKneePitch"},
        {36, "leftAnklePitch"},
        {37, "leftAnkleRoll"}
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
