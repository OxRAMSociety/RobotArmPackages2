#!/bin/bash


install_ros() {
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
}

install_workspace(){
	set -e
	# Update submodules
	git submodule update --init --recursive

	# Create the workspace
	colcon build --symlink-install

	# Setup rosdep
	# Do not fail of rosdep already set up
	sudo rosdep init 2> /dev/null || true
	rosdep update
}

install_moveit(){
	set -e
	# Install mixin
	sudo apt install python3-colcon-common-extensions -y
	sudo apt install python3-colcon-mixin -y

	# This might fail if default repo already exists
	# That is fine
	colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true
	colcon mixin update default

	# install vcstool
	sudo apt install python3-vcstool -y

	# install moeveit2
	sudo apt-get install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-moveit \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-diagnostics \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim 
}
