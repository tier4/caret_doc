# Measurement with CARET

This page explains usage of CARET with a sample application.
The sample application is located on [CARET_demos](https://github.com/tier4/CARET_demos.git) repository.

## Building application with CARET

To measure target applications' performance, the target should be built with CARET/rclcpp. CARET/rclcpp is another implementation of [ROS 2-based rclcpp](https://github.com/ros2/rclcpp) which has some additional tracepoints defined by CARET. If you have already built the target without CARET/rclcpp, you have to build the target with CARET/rclcpp.

For building the application with CARET/rclcpp, CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash` as shown below.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
source /opt/ros/galactic/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash # please keep the order after 'source /opt/ros/galactic/setup.bash'

colcon build --symlink-install --packages-up-to caret_demos
```

> Reason to build the target with CARET/rclcpp
> Some tracepoints must be added to template implementation, which is referred by rclcpp, for CARET to measure performance.
> In order to apply rclcpp which has the additional tracepoints, the target have to be built with CARET/rclcpp again.
> Therefor, CARET cannot measure performance of the application provided by Ubuntu's aptitude such as `demo_nodes_cpp`.
> If you want to measure such pre-build packages, please build them again from source code.

## Measuring the sample application with CARET

### Execution of LTTng session

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
# the measurement result will be recorded in ~/ros_ws/evaluate/e2e_sample in this sample
```

You can execute LTTng session via ROS launch system. If you are interested in this topic, please refer to [LTTng セッションの開始方法](../supplements/how_to_run_lttng_session.md).  
When you execute a LTTng session in one terminal, you have to open another terminal for executing the target application. Operating multiple terminals is laborious for users. Launch LTTng session along with application by `ros2 launch` is a reasonable way to apply CARET repeatedly.

### Launch of the target application

1. Open new terminal and run the target as shown in the following

   On the new terminal, as the target refers to CARET/rclcpp, CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash`

   ```bash
   # keep the order as below
   source /opt/ros/galactic/setup.bash
   source ~/ros2_caret_ws/install/local_setup.bash
   source ~/ros2_ws/install/local_setup.bash
   ```

2. Check whether the target uses CARET/rclcpp

   ```bash
   ldd ./build/caret_demos/end_to_end_sample  | grep rclcpp

   # librclcpp.so => /home/user_name/ros2_caret_ws/install/rclcpp/lib/librclcpp.so
   ```

   if you confronted with another result, please check which rclcpp is used for the target and your workspace's environment variables.

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

   CARET serves [trace filtering](../supplements/trace_filtering.md)(in Japanese). With configuration of trace filtering, CARET can ignore nodes and topics. This function is useful for a large application.

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

## Validating measurement result briefly

You can check whether measurement is successful or not with `babeltrace` command before analyzing result data.

```bash
# To check which tracepoints are captured as result data
$ babeltrace ~/ros2_ws/evaluate/e2e_sample/ | cut -d' ' -f 4 | sort -u
ros2:callback_end:
ros2:callback_start:
ros2_caret:add_callback_group_static_executor:
ros2_caret:callback_group_add_service:
ros2_caret:callback_group_add_subscription:
ros2_caret:callback_group_add_timer:
ros2_caret:construct_static_executor:
ros2_caret:dds_bind_addr_to_stamp:
ros2_caret:dds_write:
ros2_caret:rmw_implementation:
ros2:dispatch_subscription_callback:
ros2:rclcpp_callback_register:
ros2:rclcpp_publish:
ros2:rclcpp_service_callback_added:
ros2:rclcpp_subscription_callback_added:
ros2:rclcpp_subscription_init:
ros2:rclcpp_timer_callback_added:
ros2:rclcpp_timer_link_node:
ros2:rcl_init:
ros2:rcl_node_init:
ros2:rcl_publish:
ros2:rcl_publisher_init:
ros2:rcl_service_init:
ros2:rcl_subscription_init:
ros2:rcl_timer_init:
```

If there is loss of captured tracepoints, suspects the following.

- Outstandingly loss of `*_init` tracepoints
  - To avoid this, start LTTng session before your application is executed
- Loss of `ros2_caret:*` tracepoints
  - Check whether CARET/rclcpp is applied or not

### Tracer discarded error

`Tracer discarded` will be observed in some case, especially when applying CARET to a large application. If you find this error, CARET has failed in sampling tracepoints.

```bash
[warning] Tracer discarded 328 events between [10:46:15.566916889] and [10:46:15.620323777]
in trace UUID 353a72bc12d4bcc85c9158dd8f88ef9, at path: "end_to_end_sample/ust/uid/10368/64-bit",
within stream id 0, at relative path: "ros2_3".
You should consider recording a new trace with larger buffers or with fewer events enabled.
```

LTTng session collects sampling data generated by tracepoints. Sampling data are stored into ring-buffer as explained [LTTng documents](https://lttng.org/man/7/lttng-concepts/v2.13/#doc-channel). After a piece of ring-buffer is occupied, sampling data is stored into next empty piece while the occupied piece is copied to file. If there is no room to store sampling data in all pieces of ring-buffer, sampling data will be discarded.

You can avoid this error with the following two approach.

- to filter topics and nodes which can be ignored with trace filtering explained in the previous section
  - especially, filtering highly-frequent nodes and topics is effective
- to increase size of ring buffer defined in [`lttng_impl.py`](https://github.com/tier4/ros2_tracing/blob/2cd9d104664b4bf4d7507d01e5553129eefe1c9a/tracetools_trace/tracetools_trace/tools/lttng_impl.py#L109F)
