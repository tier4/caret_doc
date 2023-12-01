# Installation

## Requirements

CARET is confirmed to run on the platforms shown in the following table with supported version.

| dependent platform | supported version |
| ------------------ | ----------------- |
| ROS                | Humble            |
| Ubuntu             | 22.04             |
| LTTng              | stable-2.13       |
| Linux Kernel       | 5.15.x            |
| Python3            | 3.10.x            |

The recent version, after v0.3.0, of CARET supports only the combination of ROS 2 Humble and Ubuntu 22.04.  
If you want to run CARET on ROS 2 Galactic and Ubuntu 20.04, please refer to [documents for v0.2.3](https://tier4.github.io/caret_doc/refs-tags-v0.2.3/tutorials/installation/)  
We have added an experimental implementation to work with iron.
To install for iron, open `for iron` in the Installation section below and follow the instructions.

## Installation

Installation using meta repository is the least time-consuming way to install CARET.  
With meta repository and Ansible, you can skip the laborious manual setup which is explained in manual installation(./manual_installation.md) section (written in Japanese).

Please execute the following steps on Ubuntu 20.04. The order is important so that you have to follow the steps in order.

1. Clone `caret` and enter the directory.

   ```bash
   git clone https://github.com/tier4/caret.git ros2_caret_ws
   cd ros2_caret_ws
   ```

   `main` branch is dedicated for ROS 2 Humble. If you want to use CARET for ROS 2 Galactic, please execute `git checkout galactic` in `ros2_caret_ws` directory.

2. Create the src directory and clone repositories into it.

   CARET uses vcstool to construct workspaces.

   ```bash
   mkdir src
   vcs import src < caret.repos
   ```

   <details>
   <summary>for iron</summary>

   ```bash
   mkdir src
   vcs import src < caret_iron.repos
   ```

   </details>

3. Run `setup_caret.sh`.

   ```bash
   ./setup_caret.sh
   ```

   <details>
   <summary>for iron</summary>

   ```bash
   ./setup_caret.sh -d iron
   ```

   </details>

4. Build the workspace.

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   <details>
   <summary>for iron</summary>

   ```bash
   source /opt/ros/iron/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   </details>

5. Check whether CARET (ros2-tracing) is enabled.

   CARET inherits some functions from [ros2-tracing](https://gitlab.com/ros-tracing/ros2_tracing).

   ```bash
   source ~/ros2_caret_ws/install/local_setup.bash
   ros2 run tracetools status # return Tracing enabled
   ```

If you see `Tracing enabled`, you can continue to apply CARET to your application.
