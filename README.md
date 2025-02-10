# RobotArmPackages
ROS Packages related to the OxRAM Robot Arm project.

To use, `git clone --recursive https://github.com/OxRAMSociety/RobotArmPackages2.git` into the `src` folder of your workspace.

## Installation:
We recommend that you use Ubuntu 24.04 with ROS2 Jazzy Jalisco

1. Install Ubuntu 24.04 (we recommend using a VM, such as [VirtualBox](https://www.virtualbox.org/))
2. If you do not have ROS, run `./install_ros.sh` from the terminal. This script is designed to not break if run multiple times.
3. Reboot
4. Then, run `./install_workspace.sh` at the root of this repository. This script is designed to not break if it is run multiple times
5. (Optional) If you want to run computer vision, install libraries using `./install_cv_packages.sh`
6. Follow README.md for packages you will be running/developing

## Usage
Run `source ./setup.sh` when first opening in a new terminal

Run `source ./rebuild.sh` in the terminal to rebuild code

Run `./test.sh` to run the tests for the packages

<!-- ## Running the code: -->
<!-- In separate terminals -->
<!-- 1. Start the main scripts -->
<!--   ```bash -->
<!--   roslaunch rbx1_scripts rbx1_init.launch -->
<!--   ``` -->
<!---->
<!-- 2. Start the camera (select the appropriate video device and put it instead of -->
<!--    video0) -->
<!--   ```bash -->
<!--   roslaunch rbx1_computer_vision camera_init.launch video_dev:=/dev/video0 -->
<!--   ``` -->
<!---->
<!-- 3. Start YOLO -->
<!--   ```bash -->
<!--   roslaunch yolov5_ros yolov5.launch input_image_topic:=/camera/color/image_rect_color device:=cpu -->
<!--   ``` -->
<!---->
