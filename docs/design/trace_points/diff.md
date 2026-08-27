# Differences from original ROS

<prettier-ignore-start>
!!! note
    In Jazzy, all trace points required by CARET are already implemented, so there is no difference from ROS 2; however, this section has been retained to explain the historical background.
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

- [Tracepoints](./index.md)

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
