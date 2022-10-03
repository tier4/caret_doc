# Troubleshooting

## Problems during visualization, such as message flow not being displayed

The following causes are possible

- Environment variable LD_PRELOAD is not set

- No logs were recorded at the time of initialization.

  - Log with the ros2_tracing command
    Execute the ros2_tracing command by pressing the Enter key to start, then it is necessary to execute the application to be measured.

- Target application is not rebuilt with rclcpp with tracepoints added

  - caret adds some trace points to the ros layer.
    To enable these trace points, the application must be rebuilt as follows

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
    source ~/ros2_caret_ws/install/local_setup.bash
    colcon build --symlink-install
    ```

## The measurement results are wrong

The following causes are possible

- Trace results are not saved in time and are lost

  - Execute the following command and confirm that no error output is generated.

    ```bash
    babeltrace autoware > /dev/null
    [warning] Tracer discarded 3435 events between [15:05:22.967846940] and [15:05:23.025356129] in trace UUID 236d978f8bde4cbc9460b0f4e008081, at path: "autoware/ust/uid/1000/64-bit", within stream id 0, at relative path: "ros2_12". You should consider recording a new trace with larger buffers or with fewer events enabled.
    [warning] Tracer discarded 3910 events between [15:05:22.972199681] and [15:05:23.024463592] in trace UUID 236d978f8bde4cbc9460b0f4e008081, at path: "autoware/ust/uid/1000/64-bit", within stream id 0, at relative path: "ros2_6". You should consider recording a new trace with larger buffers or with fewer events enabled.
    ```

    If you see `Tracer discarded 3435 events` as above, the measurement result may not be correct.

  - If the trace results are not saved in time, set the environment variables as follows and measure again.

    ```bash
    export CARET_IGNORE_NODES="/rviz*"
    export CARET_IGNORE_TOPICS="/clock:/parameter_events"
    ```

    For more information about settings, see the [trace filtering](../recording/trace_filtering.md).

## TraceResultAnalyzeError: Failed to find

This is an error that occurs when the information in the architecture file is not found in the trace results.
It refers to a mismatch between the architecture file and the trace results.
Please review the architecture file or measurement method with reference to the error statement displayed.

Example

```text
TraceResultAnalyzeError: Failed to find callback_object.node_name: /localization/pose_twist_fusion_filter/ekf_localizer, callback_name: timer_callback_0, period_ns: 19999999, symbol: void (EKFLocalizer::?)()
```

- Callback information not found.
  - node_name：/localization/pose_twist_fusion_filter/ekf_localizer
  - callback_name：timer_callback_0
  - period_ns：19999999
  - symbol：void (EKFLocalizer::?)()

## Only certain nodes are not being measured

The phenomenon of missing some trace points, which may cause the problem.

We are currently finding a problem where trace points added to a forked ROS layer are not being measured correctly.  
This is due to the forked rclcpp not being referenced when building.

If you are missing trace points in the above procedure, please add the following line after find_package in CMakeLists.txt.

```cmake
include_directories(SYSTEM /home/autoware/ros2_caret_ws/install/rclcpp/include)
```

For tracepoints added with forked rclcpp, see [Trace Point Definition](../design/supported_tracepoints.md). The items which the implementation method is "rclcpp package new addition" are the forked added tracepoints.

## Message flow diagram stops in the middle
