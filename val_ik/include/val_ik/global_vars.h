#ifndef GLOBALVARS_H
#define GLOBALVARS_H
#include "ros/ros.h"

namespace val_ik_global{
	extern const std::vector<std::string>   drake_robot_state_names;
	extern const std::map<std::string, int> drake_state_name_to_state_index;
	extern const std::map<int, std::string> drake_state_index_to_state_name;
	void output_global_vars();
}
#endif