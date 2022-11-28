# Recording with CARET

This page explains usage of CARET with a sample application.
The sample application is located on [CARET_demos](https://github.com/tier4/CARET_demos.git) repository.

See [Recording](../recording/index.md) to find more details.

## Building application with CARET

To trace a target application, the target should be built with CARET/rclcpp. If you have already built the target without CARET/rclcpp, you have to build the target with CARET/rclcpp again. For building the application with CARET/rclcpp, CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash` as shown below.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

git clone https://github.com/tier4/CARET_demos.git src/CARET_demos

source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash # please keep the order after 'source /opt/ros/humble/setup.bash'

colcon build --symlink-install --packages-up-to caret_demos --cmake-args -DBUILD_TESTING=OFF
```

The following command allows you to check whether CARET/rclcpp is applied to each package.
If caret/rclcpp is not applied to the package you want to record, please check which rclcpp is used for the target and your workspace's environment variables.

```bash
ros2 caret check_caret_rclcpp --workspace ~/ros2_ws/

# Expected output. CARET/rclcpp is applied to all packages
INFO    : 2022-06-12 12:26:49 | All packages are built using caret-rclcpp.

# In case there are packages to which CARET/rclcpp is not applied
# The following message will be outputted
WARNING : 2022-06-12 12:25:26 | The following packages have not been built using caret-rclcpp:
   demo_nodes_cpp
   caret_demos
   intra_process_demo
```

## Tracing the sample application with CARET

### Starting LTTng session

CARET depends on LTTng for tracing applications. LTTng session has to be started before a target application runs. Note that if you execute the target application before starting LTTng session, it will result in a lack of trace points.

You can execute LTTng session, for CARET, with a simple command interface as well as ros2-tracing.

```bash
source /opt/ros/humble/setup.bash

# set a destination directory. ~/.ros/tracing is default.
mkdir -p ~/ros2_ws/evaluate
export ROS_TRACE_DIR=~/ros2_ws/evaluate

ros2 trace -s e2e_sample -k -u "ros2*"
# Start session with pressing Enter key
```

### Launching the target application

Open a new terminal and run the target as shown in the following

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
^C # Finish with Ctrl+C after several seconds

[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[end_to_end_sample-1] [INFO] [1631481246.160958444] [rclcpp]: signal_handler(signal_value=2)
[INFO] [end_to_end_sample-1]: process has finished cleanly [pid 722356]
```

You can finish the target application and LTTng session.
LTTng session will be closed if you push `Enter` key on the terminal where the LTTng session runs.

## Validating recorded data briefly

You can check whether tracing is successful or not with `ros2 caret check_ctf` command before visualizing recorded data.

```bash
ros2 caret check_ctf -d ~/ros2_ws/evaluate/e2e_sample/

# If there are problems with the recorded data, warning messages will be displayed.
```

--8<-- "includes/glossary.md"
