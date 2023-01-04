# Differences from original ROS

<prettier-ignore-start>
!!! note
    This section explains differences between v0.2 implementation of CARET and implementation of ROS 2 Galactic. The explanation is not up-to-date from viewpoints of implementation, but it is enough for readers to understand differences from viewpoints of design.
<prettier-ignore-end>


## v0.2 vs galactic

[caret.repos](https://github.com/tier4/caret/blob/main/caret.repos) contains the following repositories

- <https://github.com/ros2/rcl.git>
- <https://github.com/tier4/rclcpp/tree/galactic_tracepoint_added>
- <https://github.com/tier4/ros2_tracing/tree/galactic_tracepoint_added>

They are cloned from original ROS 2 repositories, respectively. This section describes the differences from originals.

### rcl

No source code is changed.
This package is cloned because rebuilding is necessary for enabling built-in trace points.

### rclcpp

This cloning is for adding trace point which cannot added via function hooking with LD_PRELOAD.

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

## v0.3 vs Humble

In v0.3, the trace points used in the Galactic version of CARET have been ported.
Some tracepoints have been added in Humble, but they are not currently used by CARET.
These tracepoints will be supported in a future version.
