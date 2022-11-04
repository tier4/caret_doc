# Subscription

## Inter process communication

A simplified sequence diagram focusing only on the relevant data flow is shown below.

```plantuml
@startuml
title: Definition of major tracepoints

participant UserCode
participant rclcpp
participant rcl
participant rmw
participant DDS
participant LTTng

activate DDS
activate rmw
activate rcl
activate rclcpp

DDS -> rmw
deactivate DDS

rmw -> rcl
deactivate rmw

rcl -> rclcpp
deactivate rcl


rclcpp -> LTTng: [dispatch_subscription_callback]
rclcpp -> LTTng: [callback_start]
rclcpp -> UserCode: callback start
activate UserCode
UserCode -> rclcpp: callback end
deactivate UserCode

deactivate rclcpp
@enduml
```

`to_dataframe` API returns following columns.

| Column                   | Type           | Description                                            |
| ------------------------ | -------------- | ------------------------------------------------------ |
| callback_start_timestamp | System time    | Callback start time                                    |
| message_timestamp        | Message data   | Time of header.stamp. Zero when header is not defined. |
| source_timestamp         | Depends on DDS | Timestamp to used for binding with subscription.       |

See also

- [Subscription API](https://tier4.github.io/CARET_analyze/latest/infra/#caret_analyze.infra.lttng.records_provider_lttng.RecordsProviderLttng.subscribe_records)
- [Trace points | dispatch_subscription_callback](../trace_points/runtime_trace_points.md#ros2dispatch_subscription_callback)
- [Trace points | callback_start](../trace_points/runtime_trace_points.md#ros2callback_start)

## Intra process communication

```plantuml
@startuml
title: Definition of major tracepoints

participant UserCode
participant rclcpp
participant LTTng

activate rclcpp


rclcpp -> LTTng: [dispatch_intra_subscription_callback]
rclcpp -> LTTng: [callback_start]
rclcpp -> UserCode: callback start
activate UserCode
UserCode -> rclcpp: callback end
deactivate UserCode

deactivate rclcpp
@enduml
```

`to_dataframe` API returns following columns.

| Column                   | Type                      | Description                                            |
| ------------------------ | ------------------------- | ------------------------------------------------------ |
| callback_start_timestamp | System time               | Callback start time                                    |
| message_timestamp        | Message data              | Time of header.stamp. Zero when header is not defined. |
| source_timestamp         | Depends on DDS (Optional) | NaN.                                                   |

See also

- [Subscription API](https://tier4.github.io/CARET_analyze/latest/infra/#caret_analyze.infra.lttng.records_provider_lttng.RecordsProviderLttng.subscribe_records)
- [Trace points | dispatch_intra_process_subscription_callback](../trace_points/runtime_trace_points.md#ros2dispatch_intra_process_subscription_callback)
- [Trace points | callback_start](../trace_points/runtime_trace_points.md#ros2callback_start)
