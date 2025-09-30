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
export ROS_DISTRO=jazzy
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh
echo ""
echo "MAVROS installation completed."

echo "-------------------------"
echo "Installing Custom Models"
echo "-------------------------"

pwd
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
echo ""

echo "-------------------------"
echo "Installing QGroundControl" 
echo "-------------------------"

cd ..

# Enable serial-port access Add your user to the dialout group so you can talk to USB devices without root:
sudo usermod -aG dialout "$(id -un)"

sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

URL="https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage"
OUTPUT="QGroundControl-x86_64.AppImage"

echo "Downloading QGroundControl..."
curl -L "$URL" -o "$OUTPUT"

chmod +x "$OUTPUT"
echo "Downloaded and made executable: $OUTPUT"

echo "Fertig :). Tschüssi :)"