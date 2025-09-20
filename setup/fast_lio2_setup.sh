#!/bin/bash

echo "--------------------"
echo "Installing Livox-SDK2"
echo "--------------------"

# Install livox-SDK
cd ~/
mkdir -p ws_livox/src/
cd ws_livox/
colcon build
echo "source ~/ws_livox/install/setup.bash" >> ~/.bashrc 
sudo apt install cmake
cd src/
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2/
mkdir build
cd build
echo -e "\e[32m============ Livox-SDK2 cloned successfully ============\e[0m"

########################### FIX cmake error ###########################  
DEFINE_H="$HOME/ws_livox/src/Livox-SDK2/sdk_core/comm/define.h"
FILE_MANAGER_H="$HOME/ws_livox/src/Livox-SDK2/sdk_core/logger_handler/file_manager.h"
add_include_once_after_block() {
  local file="$1"

  # If already present, do nothing
  if grep -Eq '^[[:space:]]*#include[[:space:]]*<cstdint>[[:space:]]*$' "$file"; then
    exit 0
  fi

  awk '
    BEGIN { have=0; added=0; inblock=0 }
    {
      if ($0 ~ /^[[:space:]]*#include[[:space:]]*<cstdint>[[:space:]]*$/) { have=1 }
      if ($0 ~ /^[[:space:]]*#include[[:space:]]*[<"].*[>"][[:space:]]*$/) {
        inblock=1
        print
        next
      }
      if (inblock && !have && !added) {
        print "#include <cstdint>"
        added=1
      }
      inblock=0
      print
    }
  ' "$file" > "$file.tmp" && mv "$file.tmp" "$file"
}
add_include_once_after_block "$DEFINE_H"
add_include_once_after_block "$FILE_MANAGER_H"

# ███████████████████████
echo -e "\e[32m============ CMake error fixed successfully ============\e[0m"

####################### ######################### #####################
cmake .. && make -j4
sudo make install

# ███████████████████████
echo -e "\e[32m============ livox-SDK builded successfully ============\e[0m"

echo "--------------------"
echo "Installing Livox ROS Driver 2 "
echo "--------------------"

# Install Livox ROS Driver 2 
cd ..
cd ..
cd ..
rosdep install --from-paths src --ignore-src -y
cd src/
git clone https://github.com/Livox-SDK/livox_ros_driver2.git 
source /opt/ros/jazzy/setup.sh
cd livox_ros_driver2/
./build.sh humble

# ███████████████████████
echo -e "\e[32m============ Livox ROS Driver 2 builded successfully ============\e[0m"

echo "--------------------"
echo "Installing FAST-LIO2 "
echo "--------------------"

# Install FAST-LIO2 package 
source ~/ws_livox/install/setup.bash
cd ~/
cd slam_ws/src/
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release

# ███████████████████████
echo -e "\e[32m============ FAST-LIO2 builded successfully ============\e[0m"

echo "---------------------------------"
echo "FAST-LIO2 installation completed."
echo "---------------------------------"