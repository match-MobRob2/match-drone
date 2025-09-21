#!/bin/bash

echo "Tobi ist doof"
echo "████████████████████████"
echo "████   ██      ██   ████"
echo "████                ████"
echo "████   ██      ██   ████"
echo "████     ██████     ████"
echo "████████████████████████"
echo "------------------------------"

if [[ $(lsb_release -rs) == "24.04" || $(lsb_release -rs) == "22.04" ]]; then
    echo "Ubuntu version is supported."
else
    echo "Unsupported Ubuntu version."
    exit 1
fi

echo "------------------------------"
echo "Checking for ROS 2 installation..."
echo "------------------------------"

#check if ros2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ROS 2 is not installed."
    echo ""
    echo "Installing ROS 2..."
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
    sudo dpkg -i /tmp/ros2-apt-source.deb

    sudo apt update

    sudo apt upgrade
    sudo apt install python3-colcon-common-extensions
    sudo apt install ros-jazzy-desktop

    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    # Installation commands go here
    echo "ROS 2 installation completed."
else
    echo "ROS 2 is installed."
fi


echo "------------------------"
echo "Installing PX4-Autopilot"
echo "------------------------"
pwd
cd ..
cd ..
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
touch COLCON_IGNORE
bash ./Tools/setup/ubuntu.sh
make px4_sitl 
echo "PX4-Autopilot installation completed."
echo ""

echo "------------------------"
echo "Installing MAVROS"
echo "------------------------"
cd ..
cd ..
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh
echo ""
echo "MAVROS installation completed."

echo "-------------------------"
echo "Installing Custom Models"
echo "-------------------------"

cd src/match-drone/match_models/
chmod +x install_models.sh
./install_models.sh

cd ..
cd ..
cd PX4-Autopilot
make px4_sitl

echo ""
echo "Custom Models installation completed."
echo ""

echo "-----------------"
echo "Installing ROS GZ"
echo "-----------------"

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

cd ~/slam_ws
rosdep install --from-paths src --ignore-src -y
echo -e "\e[32m============ ROS deps installed successfully ============\e[0m"

# cd src/match-drone/setup/
# pip install -r requirements.txt
# echo -e "\e[32m============ PIP deps installed successfully ============\e[0m"
