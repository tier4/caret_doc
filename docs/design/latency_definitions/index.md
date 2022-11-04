# Latency definition

CARET mainly measures the following

- Callback latency
- Communication latency
- Node latency
- Path latency

The simplified sequence diagram shown below illustrates each definition.

![Latency overview](../../imgs/latency_overview.png)

Here, the horizontal axis represents time and the vertical axis represents layers.
The red line represents message flow.
A message is received in the subscription callback and the processed data is published to the next node.
In this way, information is propagated from the sensor node to the actuator node.

CARET measures the following three main locations for latency calculation.

- Callback start
- Callback end
- Publish

Latency is calculated by calculating the time difference required for each measurement section.

For a more detailed definition, see

- [Callback](./callback.md)
- [Communication](./communication.md)
- [Node](./node.md)
- [Path](./path.md)

CARET can also check other time series data on the Publish side as well.
All the objects for which time series data can be obtained are listed below.

| Target                              | Configuration required? |
| ----------------------------------- | ----------------------- |
| [Path](./path.md)                   | Yes                     |
| [NodePath](./node.md)               | Yes                     |
| [Communication](./communication.md) | No                      |
| [Callback](./callback.md)           | No                      |
| [Publisher](./publisher.md)         | No                      |
| [Subscription](./subscription.md)   | No                      |
| [Timer](./timer.md)                 | No                      |

Here, for Path and NodePath, definitions must be given manually.
For details on setting the definitions, see [Configuration](../../configuration/index.md).

## Detailed Sequence

Below is a detailed sequence diagram of the SingleThreadedExecutor, from publish in the callback to the execution of the subscription callback.

```plantuml
@startuml
title: Definition of major tracepoints

participant UserCode
participant ROS2
participant DDS
participant LTTng

== Publisher Side ==

activate ROS2
activate UserCode
UserCode -> ROS2: publish()
activate ROS2
ROS2 -> LTTng: [rclcpp_publish] \n[rclcpp_intra_publish]


ROS2 -> DDS: dds_write()
deactivate ROS2
activate DDS
DDS -> LTTng: [dds_write]

UserCode -> ROS2 : callback_end
deactivate UserCode
deactivate ROS2
deactivate DDS


== Subscription Side ==

UserCode -> ROS2 : spin()
activate DDS
activate ROS2

loop
    ROS2 -> ROS2 : wait until next event
    activate ROS2
    DDS -> ROS2: <notify> on_data_available
    ROS2 -> LTTng : [on_data_available]
    deactivate ROS2

    deactivate DDS
    ROS2 -> DDS : reset ready-set, take messages
    activate DDS
    DDS -> ROS2
    deactivate DDS

    group execute timer callbacks
    end

    group execute subscription callbacks
        ROS2 -> LTTng: [callback_start]
        ROS2 -> UserCode: callback start
        activate UserCode
        UserCode -> ROS2: callback end
        deactivate UserCode
        ROS2 -> LTTng: [callback_end]
    end

    group execute service callbacks
    end

    group execute client callbacks
    end
end

deactivate ROS2
@enduml
```

Here, each element indicates the following

- UseCode is a callback
- ROS2 is rclcpp, rcl, and rmw
- DDS is FastDDS or CycloneDDS
- LTTng is the output destination for tracepoints

Within the spin of Subscription, the executable callbacks are executed sequentially.

In this way, the executor schedules callbacks.
If there are multiple executable callbacks, they are executed sequentially, so other callbacks may have to wait.

<prettier-ignore-start>
!!! Info
      There have been many different proposals for schedulers, and the information provided above may not be up-to-date.
      Please keep in mind that system performance will vary depending on the scheduler you choose.
<prettier-ignore-end>
