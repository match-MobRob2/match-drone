#!/bin/bash

set -euo pipefail

GREEN="$(tput setaf 2)"
RED="$(tput setaf 1)"
YELLOW="$(tput setaf 3)"
BOLD="$(tput bold)"
RESET="$(tput sgr0)"


echo "${GREEN}███╗   ███╗  █████╗  ████████╗  ██████╗  ██╗  ██╗${RESET}"
echo "${GREEN}████╗ ████║ ██╔══██╗ ╚══██╔══╝ ██╔════╝  ██║  ██║${RESET}"
echo "${GREEN}██╔████╔██║ ███████║    ██║    ██║       ███████║${RESET}"
echo "${GREEN}██║╚██╔╝██║ ██╔══██║    ██║    ██║       ██╔══██║${RESET}"
echo "${GREEN}██║ ╚═╝ ██║ ██║  ██║    ██║    ╚██████╗  ██║  ██║${RESET}"
echo "${GREEN}╚═╝     ╚═╝ ╚═╝  ╚═╝    ╚═╝     ╚═════╝  ╚═╝  ╚═╝${RESET}"

printf "\n"
printf "\n"

printf "Checking Ubuntu version...\t"
sleep 1
printf "${YELLOW}Detected version: $(lsb_release -rs)${RESET}\n"

if [ "$(lsb_release -rs)" == "22.04" ]; then
    echo "${GREEN}Ubuntu version is supported.${RESET}"
else
    echo "${RED}Unsupported Ubuntu version.${RESET}"
    exit 1
fi

printf "\n"
printf "Checking for ROS 2 installation...\t"
sleep 1

if ! command -v ros2 &> /dev/null; then
    printf "[${RED}✖${RESET}]\n"

    #Question to user
    read -p "ROS 2 is not found. Do you want to install ROS 2? (y/n): " choice
    if [[ "$choice" =~ ^[Yy]$ ]]; then
        printf "${BOLD}Installing ROS 2... ${RESET}\n"  

        sudo apt install software-properties-common -y > install.log 2>&1
        sudo add-apt-repository universe >> install.log 2>&1

        sudo apt update && sudo apt install curl -y >> install.log 2>&1
        export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') >> install.log 2>&1
        curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" >> install.log 2>&1
        sudo dpkg -i /tmp/ros2-apt-source.deb >> install.log 2>&1

        printf "Updating packages... \t"
        sudo apt update >> install.log 2>&1
        printf "[${GREEN}✔${RESET}]\n"

        printf "Upgrading packages... \t"
        sudo apt upgrade -y >> install.log 2>&1
        printf "[${GREEN}✔${RESET}]\n"

        printf "Installing ROS 2... \t"
        sudo apt install ros-humble-desktop -y >> install.log 2>&1
        sudo apt install ros-dev-tools -y >> install.log 2>&1
        printf "[${GREEN}✔${RESET}]\n"

        printf "Sourcing ROS 2 setup.bash... \t"
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        printf "[${GREEN}✔${RESET}]\n"
        printf "${GREEN}ROS 2 installation completed.${RESET}\n"
        printf "\n"

    elif [[ "$choice" =~ ^[Nn]$ ]]; then
        printf "Exiting setup.\n"
        exit 1
    else 
        echo "Exiting setup."
        exit 1
    fi
else
    printf "[${GREEN}✔${RESET}]\n"
fi

printf "\n"
printf "${BOLD}Installing PX4-Autopilot...${RESET}\n"
pwd
cd ..
git clone --branch v1.16.1 --single-branch https://github.com/PX4/PX4-Autopilot --recursive 
cd PX4-Autopilot
touch COLCON_IGNORE
bash ./Tools/setup/ubuntu.sh
export DONT_RUN=1
make px4_sitl 
printf "${GREEN}PX4-Autopilot installation completed.${RESET}\n"
printf "\n"

printf "Installing MAVROS... \n"
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O - | sudo bash 
sudo bash ./install_geographiclib_datasets.sh 

printf "Installing Custom Models"... \n"
cd ..
cd src/match_models/
chmod +x install_models.sh
./install_models.sh
cd ..
cd PX4-Autopilot
export DONT_RUN=1
make px4_sitl
printf "Installing Custom Models... [${GREEN}✔${RESET}]\n"
printf "\n"
printf "\n"
printf "Installing FAST_LIO2... \n"
cd ..
pwd
git clone 

