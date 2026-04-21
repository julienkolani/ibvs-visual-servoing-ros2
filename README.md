# IBVS Visual Servoing ROS2

Image-Based Visual Servoing (IBVS) implementation for ROS2. Computes camera velocity commands directly from image feature errors, enabling precise visual control without explicit 3D reconstruction.

## Overview

IBVS is an advanced computer vision control technique for robotics. Instead of computing 3D positions, the controller minimizes the error between current and desired visual features directly in image space.

- Configuration: Eye-In-Hand (EIH)
- Control law: interaction matrix (image Jacobian)
- Output: 6-DOF camera velocity commands

## Tech Stack

- Python
- ROS2
- OpenCV
- NumPy

## Setup

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
# Build the package
colcon build
source install/setup.bash
# Run
ros2 run <package_name> ibvs_node
```
