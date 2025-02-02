#!/bin/bash

set -e

# Update submodules
git submodule update --init --recursive

# Create the workspace
colcon build --symlink-install
