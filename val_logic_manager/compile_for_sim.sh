#!/usr/bin/env bash
source /home/stevenjj/nstrf_ws/devel_cb/setup.bash
roscd 
catkin clean ihmc_msgs val_logic_manager
roscd ihmc_ros_core/../ihmc_msgs

touch CATKIN_IGNORE
roscd val_logic_manager/include/compile_settings/
cp on_sim/compile_settings.h ../

roscd val_logic_manager
catkin build
source /home/stevenjj/nstrf_ws/devel_cb/setup.bash