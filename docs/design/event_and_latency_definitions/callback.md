# Callback

Callback latency is defined as duration between beginning and end of callback execution.
These events are represented as `callback_start` and `callback_end`, respectively.

$$
l_{\rm{callback}} = t_{\rm{callback\ end}} - t_{\rm{callback\ start}}
$$

The sequence diagram shows how CARET rclcpp picks up two events; callback_start and callback_end.

```plantuml


participant "UserCode \n Callback" as Callback
participant "rclcpp" as Rclcpp
participant LTTng

activate Rclcpp

Rclcpp -> LTTng : sample callback_start
Rclcpp -> Callback : execute callback

activate Callback
Callback -> Rclcpp
deactivate Callback
Rclcpp -> LTTng : sample callback_end
```

`to_dataframe` API returns a table which has the following columns.

| Column         | Type        | Description         |
| -------------- | ----------- | ------------------- |
| callback_start | System time | Callback start time |
| callback_end   | System time | Callback end time   |

See also

- [Trace points | Callback Start](../trace_points/runtime_trace_points.md#ros2callback_start)
- [Trace points | Callback End](../trace_points/runtime_trace_points.md#ros2callback_end)
- [RuntimeDataProvider API](https://tier4.github.io/CARET_analyze/latest/infra/#caret_analyze.infra.lttng.lttng.Lttng.compose_callback_records)
