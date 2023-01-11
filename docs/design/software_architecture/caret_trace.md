# caret_trace

caret_trace is a package that handles recording.

The role of caret_trace is as follows

- Trace point definition
- Trace data managing for runtime recording
- Adding tracepoints with hooks
- Trace filtering
- Simtime recording

See also

- [Tracepoints](../trace_points)
- [Hook](../runtime_processing/hook.md)
- [Runtime recording](../runtime_processing/runtime_recording.md)

## Class Structure

```plantuml

note left of Context : A class that manages instances; Singleton.
class Context {
    -- public --
    + get_trace_controller(): TracingController
    + get_node(): TraceNode
    + is_recording_enabled(): bool
    + ...
}

note left of HashableKeys : Tuple-like class that can calculate and compare hash values
class HashableKeys<T1, T2, ...> {
    -- public --
    .. read only ..
    + hash(): size_t
    + equal_to(...) : bool
    -- private --
    - key: T1;
    - ...
}

note left of KeysSet : Class that stores HashableKeys as a set.
class KeysSet<T> {
    -- public --
    + insert(value: T): void
    .. read only..
    + first(): T
    + ...
    -- private --
    - keys_: set<HashbleKeys<T>>
}

note left of RecordableData : Data storage class that has a record function\nData are stored as pending during recording, and are merged after the record is completed. Thread safe.
class RecordableData<KeyT> {
    -- public [writer lock] --
    + assign(function: FuncT): void
    + store(tracepoint args): bool
    + record_next_one(): bool
    + start(): void
    + reset(): void
    + ...

    -- private --
    - func_: FuncT
    - pending_set_ : KeysSet<KeyT>
    - set_ : KeysSet<KeyT>
}

note left of LttngSession : Class for manipulating LTTng sessions.
class LttngSession {
    + is_session_running(): bool
}

note left of Clock : Class for LTTng clock
class Clock {
    + now(): int64_t
}

note left of TracingController : Class for tracepoint filtering.
class TracingController {
    -- public [writer lock] --
    + add_node(address: void *): void
    + ...

    .. read only..
    + is_allowed_node(address: void *) : bool
    + ...
}

note left of DataContainer : Class that stores RecordableData for all tracepoints.
class DataContainer{
    -- public --
    + record(loop_count: uint64_t): bool

    + store_rcl_init(tracepoint_args) bool
    + ...

    + assign_rcl_init(recording_function): void
    + ...
    -- private --
    - rcl_init_: shared_ptr<RecordableData>
    - ...
}

note left of DataRecorder : Class for handling RecordableData for all trace points
class DataRecorder {
    -- public --
    + start(): void
    + reset(): void
    + record_next(): bool
    .. read only ..
    + finished(): bool
    + ...
}

note left of TraceNode : Class for controlling recording state.
class TraceNode {
    + start_callback(caret_msgs::Start)
    + end_callback(caret_msgs::End)
    + timer_callback()
}

DataContainer "1" --> "1" DataRecorder : use
DataRecorder "1" --> "0..*" RecordableData: use
RecordableData "1" o-- "1"  KeysSet
Context "1" o-- "1"  DataContainer
Context "1" o-- "1" TraceNode
Context "1" o-- "1" TracingController
Context "1" o--- "1" Clock
Context "1" o---- "1" LttngSession
TraceNode "1" --> "1" DataContainer : use
DataContainer "1" o- "0..*"  RecordableData
KeysSet "1" o-- "0..*" HashableKeys



```

## Hook function implementation

CARET adds new trace points by hooking. Also, CARET hooks trace points built into ROS2 to activate or deactivate tracing.

Here is the pseudo code for hook functions.

```C++
void ros_trace_callback_start(TRACEPOINT_ARGS) {
  // Record trace data only if current callback is allowed to record
  if (controller.is_allowed(TRACEPOINT_ARGS)) {
    tracepoint(TRACEPOINT_ARGS); // LTTng tracepoint
  }
}

void ros_trace_XXX_init(TRACEPOINT_ARGS)
{
  // Wrapper function for tracepoint.
  // This function is executed with delay.
  // This function is executed either from the record at the end of this function
  // or from TraceNode's timer callback.
  // Duplicate data are resolved with CARET_analyze.
  static auto record = [](TRACEPOINT_ARGS, now) {
    // Record trace data only if current callback is allowed to record
    if (controller.is_allowed(TRACEPOINT_ARGS)) {
      tracepoint(TRACEPOINT_ARGS, now); // LTTng tracepoint
    }
  };

  auto now = clock.now(); // Measure immediately after function call

  if (!data_container.is_assigned_XXX()) {
    data_container.assign_XXX(record);
  }

  // Store TRACEPOINT_ARGS in memory.
  data_container.store_XXX(TRACEPOINT_ARGS, now);

  record(TRACEPOINT_ARGS, now);
}

```

`ros_trace_callback_start` is an example of hook functions to trace callback start. It is a kind of trace points for collecting events related to application
s behavior, and so called runtime trace point.

`ros_trace_XXX_init` is an example of hook function to get identification of application's component. It is expected to be called when application is launched and initialized. The trace point is categorized into initialization trace point.

A set of identifications such as a callback address or node name is given as `TRACEPOINT_ARGS`.
Several kinds of initialization trace point are served, and each of them is attached to different kind of component; executor, node, callback, publisher, subscriber, and etc. For example, one is called to collect node's identification while another is called  to collect publisher's identification.

Identifications collected from different trace points share same addresses or names as elements. By connecting such identifications by the same addresses or names, CARET is able to help you to find application's structure.

<!-- Information from callback addresses to node names, etc., can be obtained by binding them to other trace point arguments. -->

See [Initialization trace points](../trace_points/initialization_trace_points.md) for details.

## clock recorder

CARET can select simtime for visualization.
The simtime can be recorded by running the simtime_recorder node, which adds trace points for simtime recording.

```bash
ros2 run caret_trace clock_recorder
```

ClockRecorder node wakes up every second and records simtime and system time.
The recorded results are used to calculate simtime from system time.
