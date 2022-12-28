# Tracepoint filtering

As measuring a target application such as Autoware composed of many nodes, the amount of data from tracing can be very large.

Trace data discarding occurs during a large amount of recording because LTTng is discard mode to minimize effect to the system.

CARET provides the ability to disable trace points associated with specific topics or nodes.
This makes it possible to exclude from recording only trace data related to rviz nodes or tf topics, allowing CARET measurements even on large systems.

This filtering function looks at the instance addresses (e.g. callback or publisher) to see if they are included in the filter.
This check is done in O1 order because it uses std::unordered_map.

```cpp
void ros_trace_callback_start(const void * callback, bool is_intra_process) {
  static auto & controller = Singleton<TracingController>::get_instance();

  // Record trace data only if current callback is allowed to record
  if (controller.is_allowed_callback(callback)) {
    ORIG_FUNC::ros_trace_callback_start(callback, is_intra_process);
  }
}
```

See also

- [caret_trace](../software_architecture/caret_trace.md)
- [Tracepoint](../trace_points/index.md)
- [Recording trace filtering](../../recording/trace_filtering.md)
