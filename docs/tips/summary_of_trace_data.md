# Summary of trace data

CARET provides CLI that displays the number of collected events, including trace data, per node, topic, or tracepoint.

## Node/topic summary

This CLI displays the number of events per node or topic.
If the number of events is too huge to handle, [trace filtering](../recording/trace_filtering.md) is a reasonable choice to exclude unnecessary nodes/topics based on the output result.
The following sample codes show command execution as examples.

```bash
# Display the number of trace points for each node
ros2 caret node_summary -d ~/ros2_ws/evaluate/e2e_sample/

---Output text as below---

=============================================
Trace creation datetime | 2022-07-16 17:34:07
Trace range             | 17:34:07 ~ 17:35:08
Trace duration          | 0:01:00
=============================================

 node_name            |   number_of_trace_points
----------------------+--------------------------
 /message_driven_node |                     4207
 /timer_driven_node   |                     3630
 /filter_node         |                     2680
 /drive_node          |                     2146
 /sensor_dummy_node   |                     2144
 /actuator_dummy_node |                     1609

# Display the number of trace points for each topic
ros2 caret topic_summary -d ~/ros2_ws/evaluate/e2e_sample/

---Output text as below---

=============================================
Trace creation datetime | 2022-07-16 17:34:07
Trace range             | 17:34:07 ~ 17:35:08
Trace duration          | 0:01:00
=============================================

 topic_name        |   number_of_trace_points
-------------------+--------------------------
 /drive            |                     2668
 /topic1           |                     2668
 /topic2           |                     2668
 /topic4           |                     2658
 /topic3           |                     2478
 /parameter_events |                       66
 /rosout           |                        6
```

## Tracepoint summary

The following command allows you to see all tracepoints included in the trace data and the number of events collected by tracepoints.

```bash
ros2 caret trace_point_summary -d ~/ros2_ws/evaluate/e2e_sample/

---Output text as below---

=============================================
Trace creation datetime | 2022-07-16 17:34:07
Trace range             | 17:34:07 ~ 17:35:08
Trace duration          | 0:01:00
=============================================

 trace_point                                       |   number_of_trace_points
---------------------------------------------------+--------------------------
 ros2:callback_end                                 |                     4216
 ros2:callback_start                               |                     4216
 ros2_caret:dds_write                              |                     2790
 ros2_caret:dds_bind_addr_to_stamp                 |                     2790
 ros2:rcl_publish                                  |                     2650
 ros2:rclcpp_publish                               |                     2650
 ros2:dispatch_subscription_callback               |                     2620
 ros2:rclcpp_subscription_callback_added           |                       44
 ros2:rclcpp_service_callback_added                |                       44
 ros2:rclcpp_callback_register                     |                       44
 ros2:rclcpp_timer_callback_added                  |                       44
 ros2_caret:callback_group_add_service             |                       36
 ros2:rcl_service_init                             |                       36
 ros2:rcl_publisher_init                           |                       17
 ros2_caret:callback_group_add_subscription        |                       11
 ros2:rcl_node_init                                |                        6
 ros2_caret:add_callback_group                     |                        6
 ros2:rcl_subscription_init                        |                        5
 ros2:rclcpp_subscription_init                     |                        5
 ros2:rcl_timer_init                               |                        3
 ros2:rclcpp_timer_link_node                       |                        3
 ros2_caret:callback_group_add_timer               |                        3
 ros2_caret:construct_executor                     |                        1
 ros2_caret:rmw_implementation                     |                        1
 ros2:rcl_init                                     |                        1
 ros2:rcl_client_init                              |                        0
 ros2:dispatch_intra_process_subscription_callback |                        0
 ros2_caret:tilde_subscribe_added                  |                        0
 ros2_caret:tilde_subscribe                        |                        0
 ros2_caret:tilde_publisher_init                   |                        0
 ros2_caret:tilde_publish                          |                        0
 ros2_caret:sim_time                               |                        0
 ros2_caret:on_data_available                      |                        0
 ros2:message_construct                            |                        0
 ros2_caret:dds_bind_addr_to_addr                  |                        0
 ros2_caret:construct_static_executor              |                        0
 ros2:rclcpp_intra_publish                         |                        0
 ros2:rcl_lifecycle_transition                     |                        0
 ros2:rcl_lifecycle_state_machine_init             |                        0
 ros2_caret:callback_group_add_client              |                        0
 ros2_caret:add_callback_group_static_executor     |                        0
 ros2_caret:tilde_subscription_init                |                        0
```

## Filtering summary ranges

Summary display of long trace data (e.g., more than 10 minutes) takes time.
The following two options allow you to filter the load range of trace data used for summary output.
In both options, the argument type is float and the unit of time is second.

- `--duration_filter [DURATION] [OFFSET]`
  - Load only this [DURATION] from [OFFSET].
- `--strip_filter [LSTRIP] [RSTRIP]`
  - Ignore trace data for specified seconds from start/end.

An example run with the filtering option is shown below.

```bash
ros2 caret trace_point_summary -d ~/ros2_ws/evaluate/e2e_sample/ --duration_filter 30 5

---Output text as below---

=============================================
Trace creation datetime | 2022-07-16 17:34:07
Trace range             | 17:34:07 ~ 17:35:08
Trace duration          | 0:01:00
Filtered trace range    | 17:34:15 ~ 17:34:45
Filtered trace duration | 0:00:29
=============================================

 trace_point                                       |   number_of_trace_points
---------------------------------------------------+--------------------------
 ros2:callback_end                                 |                     2385
 ros2:callback_start                               |                     2385
 ros2:dispatch_subscription_callback               |                     1485
 ros2:rcl_publish                                  |                     1484
 ros2_caret:dds_write                              |                     1484
 ros2_caret:dds_bind_addr_to_stamp                 |                     1484
 ros2:rclcpp_publish                               |                     1484
 ros2:rclcpp_subscription_callback_added           |                       44
 ros2:rclcpp_service_callback_added                |                       44
 ros2:rclcpp_callback_register                     |                       44
 ros2:rclcpp_timer_callback_added                  |                       44
 ros2_caret:callback_group_add_service             |                       36
 ros2:rcl_service_init                             |                       36
 ros2:rcl_publisher_init                           |                       17
 ros2_caret:callback_group_add_subscription        |                       11
 ros2:rcl_node_init                                |                        6
 ros2_caret:add_callback_group                     |                        6
 ros2:rcl_subscription_init                        |                        5
 ros2:rclcpp_subscription_init                     |                        5
 ros2:rcl_timer_init                               |                        3
 ros2:rclcpp_timer_link_node                       |                        3
 ros2_caret:callback_group_add_timer               |                        3
 ros2_caret:construct_executor                     |                        1
 ros2_caret:rmw_implementation                     |                        1
 ros2:rcl_init                                     |                        1
 ros2:rcl_client_init                              |                        0
 ros2:dispatch_intra_process_subscription_callback |                        0
 ros2_caret:tilde_subscribe_added                  |                        0
 ros2_caret:tilde_subscribe                        |                        0
 ros2_caret:tilde_publisher_init                   |                        0
 ros2_caret:tilde_publish                          |                        0
 ros2_caret:sim_time                               |                        0
 ros2_caret:on_data_available                      |                        0
 ros2:message_construct                            |                        0
 ros2_caret:dds_bind_addr_to_addr                  |                        0
 ros2_caret:construct_static_executor              |                        0
 ros2:rclcpp_intra_publish                         |                        0
 ros2:rcl_lifecycle_transition                     |                        0
 ros2:rcl_lifecycle_state_machine_init             |                        0
 ros2_caret:callback_group_add_client              |                        0
 ros2_caret:add_callback_group_static_executor     |                        0
 ros2_caret:tilde_subscription_init                |                        0
```
