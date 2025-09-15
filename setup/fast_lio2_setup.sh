#!/bin/bash

echo "--------------------"
echo "Installing FAST-LIO2"
echo "--------------------"

# Install livox-SDK
cd ~/
mkdir -p ws_livox/src/
cd /ws_livox/src/
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build

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
####################### ######################### #####################
cmake .. && make -j
sudo make install

# Install Livox ROS Driver 2 
cd ~/ws_livox/src/
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
source /opt/ros/jazzy/setup.sh
./build.sh jazzy

# Install FAST-LIO2 package 
source ~/ws_livox/install/setup.bash
cd ~/ros2_ws/src/
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select fast_lio --allow-overriding ros_gz_bridge ros_gz_image ros_gz_interfaces ros_gz_sim ros_gz_sim_demos

echo "---------------------------------"
echo "FAST-LIO2 installation completed."
echo "---------------------------------"
