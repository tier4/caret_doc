# Measurement and Tracing with CARET

This page explains usage of CARET with a sample application.
The sample application is located on [CARET_demos](https://github.com/tier4/CARET_demos.git) repository.

## Building application with CARET

To trace a target application, the target should be built with CARET/rclcpp. CARET/rclcpp is a fork of [ROS 2-based rclcpp](https://github.com/ros2/rclcpp) which has some additional tracepoints defined by CARET. If you have already built the target without CARET/rclcpp, you have to build the target with CARET/rclcpp again.

For building the application with CARET/rclcpp, CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash` as shown below.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
source /opt/ros/galactic/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash # please keep the order after 'source /opt/ros/galactic/setup.bash'

colcon build --symlink-install --packages-up-to caret_demos
```

<prettier-ignore-start>
!!!info
      Reason to build the target with CARET/rclcpp is explained here.  
      Some tracepoints must be added to template implementation, which is referred by rclcpp header files, for CARET to trace a target application.  
      In order to apply rclcpp which has the additional tracepoints, the target have to be built with CARET/rclcpp again.  
      Therefore, CARET cannot trace the application provided by Ubuntu's aptitude such as `demo_nodes_cpp`.  
      If you want to trace such pre-build packages, please build them again from source code.
<prettier-ignore-end>

## Tracing the sample application with CARET

### Starting LTTng session

CARET depends on LTTng for tracing applications. LTTng session has to be executed while a target application runs.
You can execute LTTng session, for CARET, with a simple command interface as well as ros2-tracing.

```bash
source /opt/ros/galactic/setup.bash

# set a destination directory. ~/.ros/tracing is default.
mkdir -p ~/ros2_ws/evaluate
export ROS_TRACE_DIR=~/ros2_ws/evaluate
ros2 trace -s e2e_sample -k -u "ros2*"
# Start session with pressing Enter key

# with "-s" option, you can give session name
# the trace data will be recorded in ~/ros_ws/evaluate/e2e_sample in this sample
```

Note that if you execute the target application before executing LTTng session will result in a lack of trace points.

You can execute LTTng session via ROS launch system. If you are interested in this topic, please refer to [LTTng セッションの開始方法](../tips/how_to_run_lttng_session.md).  
When you execute a LTTng session in one terminal, you have to open another terminal for executing the target application. Operating multiple terminals is laborious for users. Launch LTTng session along with application by `ros2 launch` is a reasonable way to apply CARET repeatedly.

### Launching the target application

1. Open new terminal and run the target as shown in the following

   On the new terminal, as the target refers to CARET/rclcpp, CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash`

   ```bash
   # keep the order as below
   source /opt/ros/galactic/setup.bash
   source ~/ros2_caret_ws/install/local_setup.bash
   source ~/ros2_ws/install/local_setup.bash
   ```

2. Check whether CARET/rclcpp is applied to each package

   The following command allows you to check whether CARET/rclcpp is applied to each package.
   If caret/rclcpp is not applied to the package you want to measure, please check which rclcpp is used for the target and your workspace's environment variables.

   ```bash
   # In case there are packages to which CARET/rclcpp is not applied
   ros2 caret check_caret_rclcpp --workspace ~/ros2_ws/
   # The following message will be outputted
   WARNING : 2022-06-12 12:25:26 | The following packages have not been built using caret-rclcpp:
      demo_nodes_cpp
      caret_demos
      intra_process_demo

   # In case CARET/rclcpp is applied to all packages
   ros2 caret check_caret_rclcpp --workspace ~/ros2_ws/
   INFO    : 2022-06-12 12:26:49 | All packages are built using caret-rclcpp.
   ```

3. Set `LD_PRELOAD` for adding tracepoints provided by function hook

   ```bash
   # Enable tracepoints which are defined hooked functions.
   export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
   ```

4. (Optional) Exclude nodes and topics which you are not concerned with

   ```bash
   export CARET_IGNORE_NODES="/rviz*"
   export CARET_IGNORE_TOPICS="/clock:/parameter_events"
   ```

   CARET serves [trace filtering](../tips/trace_filtering.md)(in Japanese). With configuration of trace filtering, CARET can ignore nodes and topics. This function is useful for a large application.

5. Launch the target application, demos_end_to_end_sample

   ```bash
   ros2 launch caret_demos end_to_end_sample.launch.py
   ^C # Finish with Ctrl+C after several seconds

   [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
   [end_to_end_sample-1] [INFO] [1631481246.160958444] [rclcpp]: signal_handler(signal_value=2)
   [INFO] [end_to_end_sample-1]: process has finished cleanly [pid 722356]
   ```

   You can finish the target application and LTTng session.
   LTTng session will be closed if you push `Enter` key on the terminal where the LTTng session runs.

## Validating trace data briefly

You can check whether tracing is successful or not with `ros2 caret check_ctf` command before analyzing trace data.

```bash
ros2 caret check_ctf -d ~/ros2_ws/evaluate/e2e_sample/

# If there are problems with the trace data, warning messages will be displayed.
```

<prettier-ignore-start>
!!!info
      Executing the `ros2 caret check_ctf` command for long trace data or trace data of a large application takes a long time to complete execution.
      Therefore, it is recommended to execute the `ros2 caret check_ctf` command on a short duration of trace data before collecting trace data for a long time.
<prettier-ignore-end>

### Tracer discarded error

`Tracer discarded` will be observed in some case, especially when applying CARET to a large application. If you find this error, CARET has failed in sampling tracepoints.

```bash
WARNING : 2022-04-27 08:29:08 | Tracer discarded 42 events between 1650854449589056449 and 1650854449603217243.
WARNING : 2022-04-27 08:29:14 | Tracer discarded 29 events between 1650854463006767890 and 1650854463024865609.
WARNING : 2022-04-27 08:29:14 | Tracer discarded 12 events between 1650854463026376513 and 1650854463044841704.
```

LTTng session collects sampling data generated by tracepoints. Sampling data are stored into ring-buffer as explained [LTTng documents](https://lttng.org/man/7/lttng-concepts/v2.13/#doc-channel). After a piece of ring-buffer is occupied, sampling data is stored into next empty piece while the occupied piece is copied to file. If there is no room to store sampling data in all pieces of ring-buffer, sampling data will be discarded.

You can avoid this error with the following two approach.

- to filter topics and nodes which can be ignored with trace filtering explained in the previous section
  - especially, filtering highly-frequent nodes and topics is effective
  - highly-frequent nodes/topics can be identified by checking the [summary of trace data](../tips/summary_of_trace_data.md)
- to increase size of ring buffer defined in [`lttng_impl.py`](https://github.com/tier4/ros2_tracing/blob/2cd9d104664b4bf4d7507d01e5553129eefe1c9a/tracetools_trace/tracetools_trace/tools/lttng_impl.py#L109F)
