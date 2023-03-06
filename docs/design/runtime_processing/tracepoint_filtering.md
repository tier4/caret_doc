# Tracepoint filtering

As measuring a target application such as Autoware composed of many nodes, the amount of data from tracing can be very large.

Trace data discarding occurs during a large amount of recording because LTTng is discard mode to minimize effect to the system.

CARET provides the ability to disable trace points associated with specific topics or nodes.
This makes it possible to exclude from recording only trace data related to rviz nodes or tf topics, allowing CARET measurements even on large systems.

This filtering function looks at the instance addresses (e.g. callback or publisher) to see if they are included in the filter.
This check is done in O1 order because it uses std::unordered_map.

```cpp
void ros_trace_callback_start(const void * callback, bool is_intra_process) {
  // Record trace data only if current callback is allowed to record
  if (controller.is_allowed_callback(callback)) {
    tracepoint(callback, is_intra_process); // LTTng tracepoint
  }
}
```

See also

- [caret_trace](../software_architecture/caret_trace.md)
- [Tracepoint](../trace_points/index.md)
- [Recording | trace filtering](../../recording/trace_filtering.md)

## Tracepoint filtering in DDS layer

To filter tracepoint, we need an object such as `callback` and `publisher` which contains information about node and topic. In DDS layer, such object is not available, which means we cannot filter [`dds_write`](../trace_points/runtime_trace_points.md#ros2_caretdds_write) and [`dds_bind_addr_to_stamp`](../trace_points/runtime_trace_points.md#ros2_caretdds_bind_addr_to_stamp) in the same way with the tracepoints in ROS2 layer.

To filter these tracepoints, we utilize the fact that [`rcl_publish`](../trace_points/runtime_trace_points.md#ros2rcl_publish), `dds_write` and `dds_bind_addr_to_stamp` are always recorded sequentially in the same thread. When `rcl_publish` is filtered out, subsequent `dds_write` and `dds_bind_addr_to_stamp` in the same thread can also be filtered out. When `rcl_publish` is not filtered out, subsequent `dds_write` and `dds_bind_addr_to_stamp` in the same thread cannot be filtered out either.

We utilize thread-local storage to transmit if `rcl_publish` is recorded or not.

```c++
thread_local bool ros2caret_is_rcl_publish_recorded;

void ros_trace_rcl_publish(const void * publisher_handle, const void * message)
{
  ...

  if (controller.is_allowed_publisher_handle(publisher_handle) &&
    context.is_recording_allowed())
  {
    ((functionT) orig_func)(publisher_handle, message);
    ros2caret_is_rcl_publish_recorded = true;
  } else {
    ros2caret_is_rcl_publish_recorded = false;
  }
}

void ros_trace_rmw_publish(const void * message)
{
  if (ros2caret_is_rcl_publish_recorded) {
    tracepoint(TRACEPOINT_PROVIDER, dds_write, message);
  }
}

int dds_write_impl(void * wr, void * data, long tstamp, int action)
{
  ...

  if (context.is_recording_allowed() && ros2caret_is_rcl_publish_recorded) {
    tracepoint(TRACEPOINT_PROVIDER, dds_bind_addr_to_stamp, data, tstamp);
  }

  ...
}
```
