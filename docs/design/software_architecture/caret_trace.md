# caret_trace

`caret_trace` is a package who deliver the following feature during recording.

1. Defining the tracepoints dedicated to CARET via function hooking
2. Adding state management of tracepoints via function hooking for trace filtering and runtime recording
3. Adding function of recording with `sim_time`

See also for the reference.

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

## Implementation of tracepoints with function hooking

CARET adopts function hooking mainly for adding new trace points. On the other hand, existing tracepoints, which are built in ROS2, are also re-defined by function hooking because CARET adds the function to manage tracepoint state.

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

CARET can select simulation time, represented as `sim_time`, for visualization.
`sim_time` can be recorded by running the `simtime_recorder` node, which adds trace points for `sim_time` recording.

```bash
ros2 run caret_trace clock_recorder
```

`ClockRecorder` node wakes up per second and records a pair of `sim_time` and system time.
The pair is used to convert system time to simulation time.
