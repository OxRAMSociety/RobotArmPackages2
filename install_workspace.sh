#!/bin/bash

set -e

# Update submodules
git submodule update --init --recursive

# Create the workspace
colcon build --symlink-install

# Setup rosdep
sudo rosdep init
rosdep update

# Venv, used mostly for CV
python3 -m venv .venv
source .venv/bin/activate
