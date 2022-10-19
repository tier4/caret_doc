# Differences from original ROS

## v0.2 vs galactic

In addition to the caret repository, [caret.repos](https://github.com/tier4/caret/blob/main/caret.repos) contains the following repositories

- <https://github.com/ros2/rcl.git>
- <https://github.com/tier4/rclcpp/tree/galactic_tracepoint_added>
- <https://github.com/tier4/ros2_tracing/tree/galactic_tracepoint_added>

This section describes the differences between each repository.

### rcl

No source code changes.
Cloning this package is for enabling built-in trace points by rebuilding.

### rclcpp

This cloning is for adding trace point which cannot added by LD_PRELOAD.

See also

- [Tracepoints](../trace_points/)

It's needed to add include directory of ros2_tracing.

<prettier-ignore-start>
!!! info
    Reason to add include files of ros2_tracing to rclcpp.
    LD_PRELOAD allows custom shared libraries to be loaded with priority immediately after the start of execution.  
    On the other hand, tracepoints added to the header as described above require that the tracepoint-added version of the header be loaded first during header searching at build time.  
    An include file is added to ensure that this priority is as expected.
    When the merging of tracepoints to the ros2 mainframe, the addition of the ros2_tracing include file to rclcpp is not necessary.
<prettier-ignore-end>

### ros2_tracing

This cloning is for defining tracepoints added to rclcpp.

## v0.3 vs humble

In v0.3, the trace points used in the galactic version of CARET have been ported.
Some tracepoints have been added in humble, but they are not currently supported.
These tracepoints will be supported in a future version.
