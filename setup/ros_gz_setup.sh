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
cd ~/slam_ws/src

# Download needed software
git clone https://github.com/gazebosim/ros_gz.git -b jazzy
echo -e "\e[32m============ ros package ros_gz cloned successfully ============\e[0m"
# ███████████████████████

#Install dependencies (this may also install Gazebo):
cd ~/slam_ws
rosdep install -r --from-paths src -i -y --rosdistro jazzy

echo -e "\e[32m============ dependencies installed successfully ============\e[0m"
# ███████████████████████

# Build and install into workspace
source /opt/ros/jazzy/setup.bash 
export MAKEFLAGS="-j 1"

colcon build --allow-overriding ros_gz_bridge ros_gz_image ros_gz_interfaces ros_gz_sim ros_gz_sim_demos
echo -e "\e[32m============ ros_gz builded successfully ============\e[0m"
# ███████████████████████

echo "------------------------------"
echo "ROS GZ installation completed."
echo "------------------------------"