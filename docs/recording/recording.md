# Recording

## Recording with CARET

CARET uses LTTng for tracing a target application.
This page explains two different ways for tracing: Starting LTTng session manually and Starting LTTng session using ROS launch system.

Explanation in this page assumes CARET is installed to `~/ros2_caret_ws` and the sample application used in the tutorial section is located in `~/ros2_ws`.

## Starting LTTng session manually

Two terminals are needed for this method; one for executing a target application, another for starting a LTTng session.

1. Open a terminal and launch a target application
   - Perform environment settings in the same order as below. CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash` as the target application refers to CARET/rclcpp

     ```sh
     # Environment settings (keep the order as below)
     source /opt/ros/humble/setup.bash
     source ~/ros2_caret_ws/install/local_setup.bash
     source ~/ros2_ws/install/local_setup.bash
     ```

   - Set `LD_PRELOAD` to enable tracepoints provided by function hook

     ```sh
     export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
     ```

   - (Optional) Apply [trace filtering](./trace_filtering.md). With configuration of trace filtering, CARET can ignore unnecessary nodes/topics. This function is useful for a large application

     ```sh
     # Apply filter directly
     export CARET_IGNORE_NODES="/rviz*"
     export CARET_IGNORE_TOPICS="/clock:/parameter_events"

     # Apply filter using a setting file
     source ./caret_topic_filter.bash
     ```

   - Launch the target application

     ```sh
     ros2 run caret_demos end_to_end_sample
     ```

2. Open another terminal and start a LTTng session with the following commands
   - Trace data will be stored into the directory whose path is defined as `{ROS_TRACE_DIR}/{SESSION_NAME}`.
     - `ROS_TRACE_DIR` is an environmental variable whose default value is `~/.ros/tracing`  
        (Optional) You can set `ROS_TRACE_DIR` environment variable to change a destination directory where recorded trace data will be stored
     - `SESSION_NAME` means a session name given with `-s` option for `ros2 caret record` command
   - Press "Enter" key to start a session

   ```sh
   source /opt/ros/humble/setup.bash
   source ~/ros2_caret_ws/install/local_setup.bash

   # (Optional) Set a destination directory
   mkdir -p ~/ros2_ws/evaluate
   export ROS_TRACE_DIR=~/ros2_ws/evaluate

   ros2 caret record -s e2e_sample

   # Start session with pressing Enter key
   ```

3. Press "Enter" key to stop the LTTng session in the terminal where the LTTng session runs

4. Stop the target application

<prettier-ignore-start>
!!!info
      If you'd like to start measurement immediately after launch, reverse Step 1 and Step 2.
      You can start a Lttng session first before executing a target application as well.
      However, if recording is started during launch of a target application, it may cause missing recordings.
      Please start recording either before launch is started or after launch is completed.
<prettier-ignore-end>

<prettier-ignore-start>
!!!info
      You may find that size of recorded data is strangely smaller than expected after updating LTTng to 2.13 if you apply CARET to a large application like [Autoware](https://github.com/autowarefoundation/autoware) which has hundreds of nodes. You have to suspect that maximum number of file descriptors is not enough in the case. You can check the number with `ulimit -n` command. The default maximum number is 1024, but it is not enough for the large application. You can avoid this problem by enlarging the maximum number with executing the command; `ulimit -n 65536`.
<prettier-ignore-end>

<prettier-ignore-start>
!!!info
      To trace multi-host system, you need to follow these steps on hosts which contain nodes you want to trace.
<prettier-ignore-end>

## Starting LTTng session via ROS launch

You can start LTTng session using ROS launch system. When you have started a target application in one terminal, you have to open another terminal for starting a LTTng session as explained above. Operating multiple terminals is laborious for users. Starting LTTng session along with application by ROS launch is a reasonable way to apply CARET repeatedly.

1. Create a launch file for a target application in ROS general manner if you haven't made it

   ```py
   # launch/end_to_end_sample.launch.py
   import launch
   import launch_ros.actions


   def generate_launch_description():
       return launch.LaunchDescription([
           launch_ros.actions.Node(
               package='caret_demos', executable='end_to_end_sample', output='screen'),
       ])
   ```

2. Add description to start a LTTng session

   ```py
   # launch/end_to_end_sample_with_lttng_session.launch.py
   import launch
   import launch_ros.actions
   from tracetools_launch.action import Trace


   def generate_launch_description():
       return launch.LaunchDescription([
           Trace(
               session_name='e2e_sample',
               events_kernel=[],
               events_ust=['ros2*']
           ),
           launch_ros.actions.Node(
               package='caret_demos', executable='end_to_end_sample', output='screen'),
       ])
   ```

3. Launch a target application and a LTTng session via the launch file
   - Environment settings are still needed, but all operations are performed in just one terminal

   ```sh
   source /opt/ros/humble/setup.bash
   source ~/ros2_caret_ws/install/local_setup.bash
   source ~/ros2_ws/install/local_setup.bash

   export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)

   source ./caret_topic_filter.bash

   ros2 launch caret_demos end_to_end_sample_with_lttng_session.launch.py
   ```

<prettier-ignore-start>
!!!info
      To trace multi-host system, you need to follow these steps on hosts which contain nodes you want to trace.
<prettier-ignore-end>

## Advanced: Useful settings for launch file

- The following script shows advanced settings for a launch file
- `caret_session` option is used to set a session name. If not assigned, datetime (YYYYMMDD-HHMMSS) is used
- `caret_light` option is used to add another event filter. Only events specified in `caret_event` variable are recorded. In this example, high level events, such as ros2:rclcpp layer, are recorded if `caret_light` is set to "true". This setting will be helpful to record a huge application.
  - Please refer to [tracepoint summary](./cli_tool.md#tracepoint-summary) to find which events are recorded, and modify `caret_event` as you want.

```py
# launch/end_to_end_sample_with_lttng_session.launch.py
import launch
import launch_ros.actions
from tracetools_launch.action import Trace

import sys
import datetime
from distutils.util import strtobool


def generate_launch_description():
  caret_session = ""
  caret_event = ["ros2*"]
  caret_light = True

  for arg in sys.argv:
    if arg.startswith("caret_session:="):
      caret_session = arg.split(":=")[1]
    elif arg.startswith("caret_light:="):
      try:
        caret_light = strtobool(arg.split(":=")[1]) # 0 or 1
      except:
        print("Invalid arguments 'caret_light'.")
        print("Start tracing with 'ros2*'.")

  if caret_light:
    caret_event = [
            "ros2:*callback*",
            "ros2_caret:*callback*",
            "ros2:dispatch*",
            "ros2_caret:dispatch*",
            "ros2:rclcpp*" ,
            "ros2_caret:rclcpp*" ,
            "ros2_caret:rmw*",
            "ros2:rmw_take",
            "*callback_group*",
            "ros2_caret:*executor",
            "ros2_caret:dds_bind*",
            "ros2:rcl_*init",
            "ros2_caret:rcl_*init",
            "ros2_caret:caret_init"]

  if caret_session == "":
    dt_now = datetime.datetime.now()
    caret_session = "autoware_launch_trace_" + dt_now.strftime("%Y%m%d-%H%M%S")

  return launch.LaunchDescription([
    Trace(
      session_name=caret_session,
      events_kernel=[],
      events_ust=caret_event
    ),
    launch_ros.actions.Node(
        package='caret_demos', executable='end_to_end_sample', output='screen'),
  ])
```
