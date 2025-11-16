#!/bin/bash

# Setup general ROS2 stuff
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

# Setup repo-specific things
source ./install/local_setup.bash


### OPTIONAL ###
# Update rosdep index and install dependencies
rosdep update
rosdep install --from-paths src -y --ignore-src
