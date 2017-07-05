# val_ik
ROS package for using drake's IK solver for the Valkyrie robot.
This is a hack to use drake's IK solver and does not follow best practice. However, this is sufficient for my purposes.

First ensure you have drake installed. Specifically, I am using this forked version https://github.com/stevenjj/drake/tree/my_dev_space
This repo has a tag called `working_examples` which is the last working commit with a working example of solving valkyrie with IK.


## Test Drake and Valkyrie IK
````
$ cd [drake workspace] (mine is located at ~/dev/drake_catkin_workspace/)
# Run the visualizer
$./install/bin/drake-visualizer 

# Test the IK
$./build/drake/drake/examples/Valkyrie/test/valkyrie_ik_test

# It should have passed If you have the visualizer open, you should see valkyrie in a standing position.
````

## Compile ROS node
change `set(DRAKE_CATKIN_WS /home/stevenjj/dev/drake_catkin_workspace)` in the CMakeLists.txt
````
cd [ros workspace]
catkin build
````

## Test ROS node
Run the visualizer as before, then:
````
# source your workspace
$rosrun val_ik val_ik
````

