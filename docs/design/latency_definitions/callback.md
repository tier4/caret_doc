# Callback

The callback latency is the time difference from callback start to callback end.

$$
l_{\rm{callback}} = t_{\rm{callback\ end}} - t_{\rm{callback\ start}}
$$

A simplified sequence diagram focusing only on the relevant data flow is shown below.

```plantuml


participant "UserCode \n Callback" as Callback
participant "rclcpp" as Rclcpp
participant LTTng

activate Rclcpp

Rclcpp -> LTTng : callback start
Rclcpp -> Callback : execute callback

activate Callback
Callback -> Rclcpp
deactivate Callback
Rclcpp -> LTTng : callback end
```

`to_dataframe` API returns following columns.

| Column         | Type        | Description         |
| -------------- | ----------- | ------------------- |
| callback_start | System time | Callback start time |
| callback_end   | System time | Callback end time   |

See also

- [Trace points | Callback Start](../trace_points/runtime_trace_points.md#ros2callback_start)
- [Trace points | Callback End](../trace_points/runtime_trace_points.md#ros2callback_end)
- [RuntimeDataProvider API](https://tier4.github.io/CARET_analyze/latest/infra/#caret_analyze.infra.lttng.lttng.Lttng.compose_callback_records)
