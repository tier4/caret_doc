# Recording with CARET

This page explains usage of CARET with a sample application.
The sample application is located on [caret_demos](https://github.com/tier4/caret_demos.git) repository.

See [Recording](../recording/index.md) to find more details.

## Building application with CARET

To trace a target application, the target should be built with CARET/rclcpp. If you have already built the target without CARET/rclcpp, you have to build the target with CARET/rclcpp again. For building the application with CARET/rclcpp, CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash` as shown below.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

git clone https://github.com/tier4/caret_demos.git src/caret_demos

source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash # please keep the order after 'source /opt/ros/humble/setup.bash'

colcon build --symlink-install --packages-up-to caret_demos --cmake-args -DBUILD_TESTING=OFF
```

<details>
<summary>for iron</summary>

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

git clone https://github.com/tier4/CARET_demos.git src/CARET_demos

source /opt/ros/iron/setup.bash

colcon build --symlink-install --packages-up-to caret_demos --cmake-args -DBUILD_TESTING=OFF
```

</details>

The following command allows you to check whether CARET/rclcpp is applied to each package.
If caret/rclcpp is not applied to the package you want to record, please check which rclcpp is used for the target and your workspace's environment variables.

```bash
ros2 caret check_caret_rclcpp ~/ros2_ws/

# Expected output. CARET/rclcpp is applied to all packages
INFO    : 2022-06-12 12:26:49 | All packages are built using caret-rclcpp.

# In case there are packages to which CARET/rclcpp is not applied
# The following message will be outputted
WARNING : 2022-06-12 12:25:26 | The following packages have not been built using caret-rclcpp:
   demo_nodes_cpp
   caret_demos
   intra_process_demo
```

<details>
<summary>for iron</summary>

There is no need to run following command.

```bash
ros2 caret check_caret_rclcpp ~/ros2_ws/
```

CARET does not require a build using caret-rclcpp with ROS 2 Distributions after iron.

</details>

## Tracing the sample application with CARET

### Launching the target application

Run the target as shown in the following.

```bash
# Environment settings (keep the order as below)
source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash

# Enable tracepoints which are defined hooked functions.
export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)

# (Optional) Exclude nodes and topics which you are not concerned with
export CARET_IGNORE_NODES="/rviz*"
export CARET_IGNORE_TOPICS="/clock:/parameter_events"

# Launch the target application, demos_end_to_end_sample
ros2 launch caret_demos end_to_end_sample.launch.py
```

<details>
<summary>for iron</summary>

```bash
# Environment settings (keep the order as below)
source /opt/ros/iron/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash

# Enable tracepoints which are defined hooked functions.
export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)

# (Optional) Exclude nodes and topics which you are not concerned with
export CARET_IGNORE_NODES="/rviz*"
export CARET_IGNORE_TOPICS="/clock:/parameter_events"

# Launch the target application, demos_end_to_end_sample
ros2 launch caret_demos end_to_end_sample.launch.py
```

</details>

### Starting recording

Open a new terminal and record the performance data.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash

# set a destination directory. ~/.ros/tracing is default.
mkdir -p ~/ros2_ws/evaluate
export ROS_TRACE_DIR=~/ros2_ws/evaluate

ros2 caret record -s e2e_sample

# Start recording with pressing Enter key
# > All process tarted recording.
# > press enter to stop...
```

<details>
<summary>for iron</summary>

```bash
source /opt/ros/iron/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash

# set a destination directory. ~/.ros/tracing is default.
mkdir -p ~/ros2_ws/evaluate
export ROS_TRACE_DIR=~/ros2_ws/evaluate

ros2 caret record -s e2e_sample

# Start recording with pressing Enter key
# > All process tarted recording.
# > press enter to stop...
```

</details>

## Validating recorded data briefly

You can check whether tracing is successful or not with `ros2 caret check_ctf` command before visualizing recorded data.

```bash
ros2 caret check_ctf ~/ros2_ws/evaluate/e2e_sample/

# If there are problems with the recorded data, warning messages will be displayed.
```
