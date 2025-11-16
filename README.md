# RobotArmPackages
ROS Packages related to the OxRAM Robot Arm project.

To use, `git clone --recursive https://github.com/OxRAMSociety/RobotArmPackages2.git`. Use the newly created folder as a workspace.

## Installation:
We recommend that you use Ubuntu 24.04 with ROS2 Jazzy Jalisco

1. Install Ubuntu 24.04 (we recommend using a VM, such as [VirtualBox](https://www.virtualbox.org/))
2. Clone this repository into a folder, `cd` into the folder.
3. Run `./install.py first-install`. It should finish without errors and tell you to reboot.
4. Reboot.
5. Run `./install.py first-install` again
6. (Optional) run `./install.py setup_cv_training` if you will be training the CV model or `./install.py setup_moveit` if you will be working with moveit

## Usage
Run `source ./setup.sh` when first opening in a new terminal

Run `source ./rebuild.sh` in the terminal to rebuild code

Run `./test.sh` to run the tests for the packages

It is safe to re-run `./install.py reinstall` multiple times

<!-- ## Running the code: -->

## Contribution workflow:
See [this comment](https://github.com/OxRAMSociety/RobotArm/issues/14#issuecomment-2614327934) for a high-level overview of the workflow.

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
