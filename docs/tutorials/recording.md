# Recording with CARET

This page explains usage of CARET with a sample application.
The sample application is located on [caret_demos](https://github.com/tier4/caret_demos.git) repository.

See [Recording](../recording/index.md) to find more details.

## Building application with CARET

To trace a target application, simply build the target with ROS 2. CARET provides tracepoints at runtime via `LD_PRELOAD`, so you do **not** need to build the application with a forked rclcpp.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

git clone https://github.com/tier4/caret_demos.git src/caret_demos

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-up-to caret_demos --cmake-args -DBUILD_TESTING=OFF
```

## Tracing the sample application with CARET

### Launching the target application

Run the target as shown in the following.

```bash
# Environment settings
~/ros2_caret_ws/setenv_caret.bash

# source caret_demos
source ~/ros2_ws/install/local_setup.bash

# Launch the target application, demos_end_to_end_sample
ros2 launch caret_demos end_to_end_sample.launch.py
```

### Starting recording

Open a new terminal and record the performance data.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash

# set a destination directory. ~/.ros/tracing is default.
mkdir -p ~/ros2_ws/evaluate
export ROS_TRACE_DIR=~/ros2_ws/evaluate

ros2 caret record -s e2e_sample

# Start recording with pressing Enter key
# > All process tarted recording.
# > press enter to stop...
```

## Validating recorded data briefly

You can check whether tracing is successful or not with `ros2 caret check_ctf` command before visualizing recorded data.

```bash
ros2 caret check_ctf ~/ros2_ws/evaluate/e2e_sample/

# If there are problems with the recorded data, warning messages will be displayed.
```
