# FAQ

## Installation

### Setup fails in ROS 2 Galactic, Ubuntu 20.04

In case you encounter errors during setup or build process, please make sure to use `galactic` branch.

```bash
git clone https://github.com/tier4/caret.git -b galactic ros2_caret_ws
cd ros2_caret_ws
mkdir src
vcs import src < caret.repos
./setup_caret.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### CLI tool doesn't work

In case CLI tool execution fails, please make sure to perform CARET environment settings.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash

ros2 caret check_caret_rclcpp --workspace <path-to-workspace>
```
