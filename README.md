# You may have to run the following commands
cd /workspace_location # same directory as build, devel, logs, and src
rosdep install --from-paths src/ --ignore-src --rosdistro=${ROS_DISTRO} -y
