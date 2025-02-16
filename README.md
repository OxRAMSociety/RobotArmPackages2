# RobotArmPackages
ROS Packages related to the OxRAM Robot Arm project.

To use, `git clone --recursive https://github.com/OxRAMSociety/RobotArmPackages2.git`. Use the newly created folder as a workspace.

## Installation:
We recommend that you use Ubuntu 24.04 with ROS2 Jazzy Jalisco

1. Install Ubuntu 24.04 (we recommend using a VM, such as [VirtualBox](https://www.virtualbox.org/))
2. If you do not have ROS, run `./install_ros.sh` from the terminal. This script is designed to not break if run multiple times.
3. Reboot
4. Follow README.md for packages you will be running/developing. For example, you will need the camera configuration files sorted out to be able to compile `computer_vison`
5. Then, run `./install_workspace.sh` at the root of this repository. This script is designed to not break if it is run multiple times
6. (Optional) If you want to run computer vision, install libraries using `./install_cv_packages.sh`

## Usage
Run `source ./setup.sh` when first opening in a new terminal

Run `source ./rebuild.sh` in the terminal to rebuild code

Run `./test.sh` to run the tests for the packages

The packages have a README.md file which explains what they do and how to get them set up.

## Helpful resources for ROS2 and git
There is an [introduction document](https://github.com/OxRAMSociety/RobotArm/blob/main/resources.md) in the RobotArm repository.

## Contribution workflow:
See [this comment](https://github.com/OxRAMSociety/RobotArm/issues/14#issuecomment-2613048568) for an explanation how to figure out what tasks to work on.
See [this comment](https://github.com/OxRAMSociety/RobotArm/issues/14#issuecomment-2614327934) for a high-level overview of the workflow, i.e. how to use git productively.

Here are the specific commands that need to be run, for each of the points:

1. Use Github
2. First, make sure that you are in the main branch. If this command returns an error, you probably have uncommitted changes.

```
git checkout main
```

Create a new branch and move to it
```
git checkout -b <number>
```
3. While developing your code, don't forget to periodically create commits with messages explaining what you did in the commit. It might be easier to use a git helper in your IDE.

When you are ready, make sure you have committed everything you want to push and push your changes to Github
```
git push
```

Then use the Github website to open a pull request.

4. Use Github
5. Same commands as in point 3, but don't forget to `git pull` and `git push` regularly to reduce the probability of conflicts
6. Ask one of us to approve your pull request


