# Tracepoints definition

This section lists all tracepoints and their definition.

Some tracepoints are used for collecting identification of executors, nodes, callbacks, and topics during application's initialization.
They are called initialization tracepoints. The other tracepoints are embedded for sampling timestamps after completion of initialization, and called runtime tracepoints.

Each tracepoint for CARET is added by followings method.

![builtin_and_extended_tracepoints](../../imgs/builtin_and_extended_trace_points.drawio.png)

- Built-in tracepoints
  - tracepoints embedded in original ROS 2 middleware which are utilized by ros2-tracing
  - some of tracepoints, for service, action and lifecycle node, are not utilized by current CARET
- Hooked tracepoints
  - CARET-dedicated tracepoints introduced by function hooking with LD_PRELOAD
- Extended tracepoints
  - CARET-dedicated tracepoints added to the fork of rclcpp

CARET utilizes some of the tracepoints built-in original ROS 2.
Some of the tracepoints are added by hooking with LD_PRELOAD, and rest trace points are added to the fork of ROS 2's rclcpp.

<prettier-ignore-start>
!!! info
    Please read this section if you are interested in CARET-dedicated tracepoints are extended by the forked rclcpp and LD_PRELOAD. CARET would like to add tracepoints by function hooking as possible. LD_PRELOAD is reasonable to hook functions defined in dynamic library, but it cannot be applied to functions by implemented with C++ template. Such template-based implementation is mapped into binary file after it is built or compiled. Builtin rclcpp uses C++ template for some functions like intra-process communication, for example. The forked rclcpp is introduced to add tracepoints to the functions.
<prettier-ignore-end>
