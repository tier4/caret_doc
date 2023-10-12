# Hook

Function hooking is one of the key techniques performed by CARET.
This section describes the function hooking introduced to CARET.

See also

- [caret_trace](../software_architecture/caret_trace.md)

## Advantage of Hook

ROS 2 is being developed separately from DDS thanks to RMW.
On the other hand, each implementation may be developed for different purposes, which makes it difficult to achieve consistent evaluation of all layers, including DDS.
CARET handles these layers across by hooks to add and manage trace points consistently.

![trace_points](../../imgs/control_trace_points_via_hook.drawio.png)

See also

- [Tracepoints definition](../trace_points/index.md)

<prettier-ignore-start>
!!!Info
    If possible, it is better to add tracepoints as built-in rather than hooks for users.
    However, CARET's priority is to evaluate software running on the current version of ROS rather than to gradually add trace points.
    For this reason, we have adopted function hooking that allows users to add tracepoints in a flexible manner.
<prettier-ignore-end>

<prettier-ignore-start>
!!!Info
    The advantage of being able to handle all layers across the board is not well utilized in the current CARET.
    In the future, we plan to use thread local memory to reduce trace points.
<prettier-ignore-end>

## LD_PRELOAD

### Advantage of LD_PRELOAD

LD_PRELOAD can be hooked even if the function is not exposed externally as an API.

The trace points themselves, which are built into the ROS layer, can also be hooked.
This enables trace filtering.

Though you might come up with using eBPF to hook, eBPF requires a context switch from user space to kernel space.
Hooking with LD_PRELOAD can be completed in user space, reducing the overhead.

See also

- [Tracepoint filtering](./tracepoint_filtering.md)

### Limits of LD_PRELOAD

There are some cases that cannot or are difficult to hook with LD_PRELOAD.

- Functions with many symbols by cpp template
- Hooks for functions that are expanded as inline code
- Hooks for functions implemented in headers

Specifically, intra-process publish cannot be hooked by LD_PRELOAD.
In CARET, trace points for intra-process communication are added in the forked rclcpp.
