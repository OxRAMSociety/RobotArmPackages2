#!/bin/bash

set -e

# Install needed packages
sudo apt install git build-essential -y

# Set up locales
sudo apt update -y
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable repositories
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update repos after a new one was added
sudo apt update -y

# Update the system
sudo apt upgrade -y

# Install ros
sudo apt install ros-dev-tools -y
sudo apt install ros-jazzy-desktop -y

# Run setup script on shell start
grep -qF "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# Tell the user to not forget to run the setup script
grep -qF "Remember to run 'source setup.sh'" ~/.bashrc || echo "echo -e \"\033[0;31mRemember to run 'source setup.sh'\033[0;0m\"" >> ~/.bashrc

# Install python-venv
sudo apt install python3.12-venv -y


