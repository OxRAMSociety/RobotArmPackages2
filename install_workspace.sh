#!/bin/bash

set -e

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
