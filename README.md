Robotic System Development for Automated Data Collection in Precision Agriculture

Summer 2025 REU, Industrial and Systems Engineering

Author: Junyang Guan

Advisor: Professor Girish Krishnan (gkrishna@illinois.edu)

Collaborated with PhD Student: James Nam (sn29@illinois.edu)

I. Introduction

The purpose of this research activity was to participate in building a precision agriculture robot that can operate autonomously. This robot is designed to operate in high tunnels, a relatively unmanaged and cluttered farming environment. Therefore, specialized algorithms that adapt to unstructured environments are needed for operation. The functionality developed during this project enables the robot to stop at regular intervals and perform automated data collection tasks using a combination of cameras and a robotic arm. In the functional perspective, this robot is composed of a robotic arm, an eye-in-hand camera, a mobile wheeled base, and a base camera on a pan/tilt mount.

Specifically, the work focused on programming the communications using the Robot Operating System 2 (ROS2) middleware. Building on previous work, the project improved the robustness of motor and arm controls, image processing, and the navigation system. The first phase involved learning the software architecture of robots and becoming familiar with working on real hardware. After that, work continued on building the communications for the robot and integrating sensing, control, and navigation components for automated data collection.

Repository overview
- This repository is a ROS workspace containing multiple packages used for perception, control, and navigation.
- Top-level directories of interest: `src/`, `build/`, `install/`, and `log/`.

Requirements
- Ubuntu 22.04 (recommended) or another Linux distribution supported by ROS2
- ROS2 distribution (e.g., Humble) installed and sourced
- `colcon` build tool

Quickstart
1. Install ROS2 following the official instructions for your distribution and chosen ROS2 release.
2. From this workspace root:

```bash
# source your ROS2 install (example: replace <ros-distro> as appropriate)
source /opt/ros/<ros-distro>/setup.bash

# build the workspace
colcon build --symlink-install

# source the overlay
source install/setup.bash
```

3. Launch nodes or packages using the package-specific launch files (see package README files under `src/`). Example usage varies by package.

Notes about nested repositories
During repository cleanup, nested `.git` directories (for example `src/` and `src/xarm_ros2`) were removed so that their contents are tracked by this main repository and visible on GitHub. If you previously had nested repositories and want to preserve them as submodules, recreate them or add as submodules explicitly.

Development notes
- Each package under `src/` generally contains its own README with package-specific instructions—see those files for details on individual packages.
- Use `git status` after adding files to verify changes before committing.

License
This project is licensed under the terms in the `LICENSE` file included with this repository.

Contact
- Junyang Guan (project author, jg73@illinois.edu)
