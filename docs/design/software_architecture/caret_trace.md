# caret_trace

caret_trace is a package that handles recording such as adding trace points.

The role of caret_trace is as follows

- Race Point Definition
- Adding Tracepoints with Hooks
- trace filtering
- simtime recording

See also

- [Tracepoints](../trace_points)
- [Hook](../runtime_processing/hook.md)

## Class Structure

```plantuml

class HashableKeys<T1, T2, ...> {
    + insert(args): void
    + has(): bool
    + hash(): size_t
}

class KeysSet<T1, T2, ...> {
    + insert(args): void
    + has(args): void
}

class Singleton<T> {
    + get_instance(): T
}

class TracingController {
    + add_node(args): void
    + is_allowed_node(args) : bool
}

class ClockRecoerder {
}

KeysSet o-- HashableKeys
```

## Hook function implementation

In addition to hooking and for adding trace points, CARET also hooks trace points built into ROS2.

Here is an example of a typical hook implementation.

```C++
void ros_trace_rcl_node_init(
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace)
{
  static auto & controller = Singleton<TracingController>::get_instance();

  // Bind node handle and node name
  controller.add_node(node_handle, ns + node_name);

  // Record trace data only if current node is allowed to record
  if (controller.is_allowed_node(node_handle)) {
    ORIG_FUNC::ros_trace_rcl_node_init)(node_handle, rmw_handle, node_name, node_namespace);
  }
}

void ros_trace_callback_start(const void * callback, bool is_intra_process) {
  static auto & controller = Singleton<TracingController>::get_instance();

  // Record trace data only if current callback is allowed to record
  if (controller.is_allowed_callback(callback)) {
    ORIG_FUNC::ros_trace_callback_start(callback, is_intra_process);
  }
}
```

Here, debugging logs and other information are omitted.

Information from callback addresses to node names, etc., can be obtained by binding them to other trace point information.
See [Initialization trace points](../trace_points/initialization_trace_points.md) for details.

## clock recorder

CARET can select simtime for visualization.
The simtime can be recorded by running the simtime_recorder node, which adds trace points for simtime recording.

```bash
ros2 run caret_trace clock_recorder
```

ClockRecorder node wakes up every second and records simtime and system time.
The recorded results are used to calculate simtime from system time.
