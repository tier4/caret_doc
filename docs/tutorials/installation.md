# Installation

## Requirements

CARET is confirmed to run on the platforms shown in the following table with supported version.

| dependent platform | supported version |
| ------------------ | ----------------- |
| ROS                | Galactic          |
| Ubuntu             | 20.04             |
| LTTng              | stable-2.12[^1]   |
| Linux Kernel       | 5.13.x            |
| Python3            | 3.8.x             |

[^1]: Planned to support LTTng 2.13 or newer for CARET v.0.3.x.

## Installation

Installation using meta repository is the least time-consuming way to install CARET.  
With meta repository and Ansible, you can skip the laborious manual setup which is explained in [tips/manual installation](../tips/manual_installation.md) section (written in Japanese).

Please execute the following steps on Ubuntu 20.04. The order is important so that you have to follow the steps in order.

1. Clone `caret` and enter the directory.

   ```bash
   git clone https://github.com/tier4/caret.git ros2_caret_ws
   cd ros2_caret_ws
   ```

2. Create the src directory and clone repositories into it.

   CARET uses vcstool to construct workspaces.

   ```bash
   mkdir src
   vcs import src < caret.repos --recursive
   ```

3. Run `setup_caret.sh`.

   ```bash
   ./setup_caret.sh
   ```

4. Build the workspace.

   ```bash
   source /opt/ros/galactic/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

5. Check whether CARET (ros2-tracing) is enabled.

   CARET inherits some functions from [ros2-tracing](https://gitlab.com/ros-tracing/ros2_tracing).

   ```bash
   source ~/ros2_caret_ws/install/local_setup.bash
   ros2 run tracetools status # return Tracing enabled
   ```

If you see `Tracing enabled`, you can continue to apply CARET to your application.
