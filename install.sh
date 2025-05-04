#!/bin/bash

set -e

install_ros() {
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
}

install_workspace(){
	# Update submodules
	git submodule update --init --recursive

	# Create the workspace
	colcon build --symlink-install

	# Setup rosdep
	# Do not fail of rosdep already set up
	sudo rosdep init 2> /dev/null || true
	rosdep update

	# Venv, used mostly for CV
	python3 -m venv .venv
	source .venv/bin/activate
}

install_cv() {
	source .venv/bin/activate
	pip3 install roboflow ultralytics
}

install_moveit(){
	# Install mixin
	sudo apt install python3-colcon-common-extensions -y
	sudo apt install python3-colcon-mixin -y
	colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
	colcon mixin update default

	# install vcstool
	sudo apt install python3-vcstool -y
}


install_ros_flag=""
install_workspace_flag=""
install_cv_flag=""
install_moveit_flag=""

# Handle flags
# https://stackoverflow.com/questions/7069682/how-to-get-arguments-with-flags-in-bash#21128172
while test $# -gt 0; do
	case "$1" in
		--all)
			shift
			install_ros_flag="y"
			install_workspace_flag="y"
			install_cv_flag="y"
			install_moveit_flag="y"
			;;
		--initial-setup)
			shift
			install_ros_flag="y"
			install_workspace_flag="y"
		;;
		--ros)
			shift
			install_ros_flag="y"
		;;
		--workspace)
			shift
			install_workspace_flag="y"
		;;
		--cv)
			shift
			install_cv_flag="y"
		;;
		--moveit)
			shift
			install_moveit_flag="y"
		;;
		*)
			echo "$1 is not a recognized flag!"
			return 1;
		;;
	esac
done

if [ -n "$install_ros_flag" ]; then
	install_ros
fi
if [ -n "$install_workspace_flag" ]; then
	install_workspace
fi
if [ -n "$install_cv_flag" ]; then
	install_cv
fi
if [ -n "$install_moveit_flag" ]; then
	install_moveit
fi
