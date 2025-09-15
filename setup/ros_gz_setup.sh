#!/bin/bash

echo "-----------------"
echo "Installing ROS GZ"
echo "-----------------"

# The ros_gz library makes heavy use of templates which causes compilers to consume a lot of memory. 
# If your build fails with c++: fatal error: Killed signal terminated program cc1plus try building with 
# "colcon build --parallel-workers=1 --executor sequential"
# You might also have to set export MAKEFLAGS="-j 1" before running colcon build to limit the number of 
# processors used to build a single package.

# Clone repo inside your ros2 workspace 
cd ~/ros2_ws/src

# Download needed software
git clone https://github.com/gazebosim/ros_gz.git -b jazzy

#Install dependencies (this may also install Gazebo):
cd ~/ros2_ws
rosdep install -r --from-paths src -i -y --rosdistro jazzy

# Build and install into workspace
source /opt/ros/jazzy/setup.bash 
export MAKEFLAGS="-j 1"
colcon build

echo "------------------------------"
echo "ROS GZ installation completed."
echo "------------------------------"
