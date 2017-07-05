# NSTRF 2017
Contains code I used for my NSTRF at NASA JSC during the summer of 2017

## Dependency Compilation
This repo contains a lot of external dependencies. Most notably to openpose_ros and darknet. Add or Remove dependencies as needed.

## Resolving ROS dependencies
You may have to run the following commands:
If you have not updated your rosdep source lists yet
````
sudo rosdep init
rosdep update
````
Then use rosdep to install missing dependencies:
````
cd /workspace_location # same directory as build, devel, logs, and src
rosdep install --from-paths src/ --ignore-src --rosdistro=${ROS_DISTRO} -y 
# Change src/ to a specific ros package directory as needed
````