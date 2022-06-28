# Summary of trace data

CARET provides CLI that displays the number of trace points for each node/topic and the number of each trace point from the trace data.

## node/topic summary

This CLI displays the number of trace points for each node/topic from the trace data.
To prevent lost traces, the number of trace points can be reduced by [trace filtering](trace_filtering.md) unnecessary nodes/topics based on the output results.
Examples of command execution are shown below.

```bash
# Display the number of trace points for each node
$ ros2 caret node_summary -d ~/ros2_ws/evaluate/e2e_sample/

---Output text as below---

node_name            |   number_of_trace_points
---------------------+--------------------------
/message_driven_node |                      600
/timer_driven_node   |                      535
/drive_node          |                      373
/filter_node         |                      371
/sensor_dummy_node   |                      371
/actuator_dummy_node |                      200
unkown               |                       16

# Display the number of trace points for each topic
$ ros2 caret topic_summary -d ~/ros2_ws/evaluate/e2e_sample/

---Output text as below---

topic_name        |   number_of_trace_points
------------------+--------------------------
unkown            |                      691
/topic3           |                      391
/drive            |                      343
/topic1           |                      343
/topic2           |                      341
/topic4           |                      339
/rosout           |                        6
```

## tracepoint summary

The following command allows you to see all trace points included in the trace data and the number of each trace point.

```bash
$ ros2 caret trace_point_summary -d ~/ros2_ws/evaluate/e2e_sample/

 trace_point                                       |   number_of_trace_points
---------------------------------------------------+--------------------------
 ros2:rclcpp_timer_callback_added                  |                       44
 ros2:rclcpp_subscription_callback_added           |                       44
 ros2:rclcpp_service_callback_added                |                       44
 ros2:rclcpp_callback_register                     |                       44
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
 ros2_caret:dds_bind_addr_to_stamp                 |                        0
 ros2_caret:dds_write                              |                        0
 ros2_caret:tilde_subscribe                        |                        0
 ros2_caret:tilde_publish                          |                        0
 ros2_caret:dds_bind_addr_to_addr                  |                        0
 ros2_caret:sim_time                               |                        0
 ros2_caret:construct_static_executor              |                        0
 ros2_caret:on_data_available                      |                        0
 ros2_caret:tilde_subscribe_added                  |                        0
 ros2_caret:tilde_publisher_init                   |                        0
 ros2:callback_end                                 |                        0
 ros2_caret:callback_group_add_client              |                        0
 ros2_caret:add_callback_group_static_executor     |                        0
 ros2:callback_start                               |                        0
 ros2:rclcpp_publish                               |                        0
 ros2:rclcpp_intra_publish                         |                        0
 ros2:rcl_publish                                  |                        0
 ros2:rcl_lifecycle_transition                     |                        0
 ros2:rcl_lifecycle_state_machine_init             |                        0
 ros2:rcl_client_init                              |                        0
 ros2:message_construct                            |                        0
 ros2:dispatch_subscription_callback               |                        0
 ros2:dispatch_intra_process_subscription_callback |                        0
 ros2_caret:tilde_subscription_init                |                        0
```
