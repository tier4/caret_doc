"Initialization tracepoints” are tracepoints that occur mainly when ROS2 configuration objects such as Node, Executor, and Callback are created.
Some tracepoints share same addresses (e.g. node_handle and callback address).
By binding these addresses, CARET constructs structures of each trace point relationship.

### Relationships for each initialization trace points

Relationships of each trace point related to a single node are shown as follows.

```mermaid
erDiagram
 rcl_init{
 address context_handle
 }

 caret_init{
 int64_t clock_offset
 string distribution
 }

 rcl_node_init{
 address node_handle
 address rmw_handle
 string node_name
 string node_namespace
 }

 rcl_publisher_init{
 address publisher_handle
 address node_handle
 address rmw_publisher_handle
 string topic_name
 size_t queue_depth
 }

 rcl_subscription_init{
 address subscription_handle
 address node_handle
 address rmw_subscription_handle
 string topic_name
 size_t queue_depth
 }

 rclcpp_subscription_init{
 address subscription_handle
 address subscription
 }

 rclcpp_subscription_callback_added{
 address subscription
 address callback
 }

 rcl_service_init{
 address service_handle
 address node_handle
 address rmw_service_handle
 string service_name
 }

 rclcpp_service_callback_added{
 address service_hadle
 address callback
 }

 rcl_timer_init{
 address timer_handle
 int64_t period
 }

 rclcpp_timer_callback_added{
 address timer_handle
 address callback
 }

 rclcpp_timer_link_node{
 address timer_handle
 address node_handle
 }

 rclcpp_callback_register{
 address callback
 string function_symbol
 }

 rclcpp_buffer_to_ipb{
 address buffer
 address ipb
 }

 rclcpp_ipb_to_subscription{
 address ipb
 address subscription
 }

 rclcpp_construct_ring_buffer{
 address buffer
 uint64_t capacity
 }

 rmw_implementation{
 string rmw_impl
 }

 rcl_client_init{
 address client_handle
 address node_handle
 address rmw_client_handle
 string service_name
 }

 rcl_lifecycle_state_machine_init{
 address node_handle
 address state_machine
 }

    rcl_node_init ||--o{ rcl_publisher_init : node_handle
    rcl_node_init ||--o{ rcl_subscription_init : node_handle
    rcl_node_init ||--o{ rclcpp_timer_link_node : node_handle
    rcl_node_init ||--o{ rcl_service_init : node_handle

    rcl_publisher_init ||--|| PUBLISHER_HANDLE : node_handle
    rcl_subscription_init ||--|| SUBSCRIPTION_HANDLE : node_handle
    rcl_timer_init ||--|| TIMER_HANDLE : node_handle
    rcl_service_init ||--|| SERVICE_HANDLE : node_handle

    rclcpp_subscription_init ||--|| rclcpp_ipb_to_subscription : subscription
    rclcpp_ipb_to_subscription ||--|| rclcpp_buffer_to_ipb : ipb
    rclcpp_buffer_to_ipb ||--|| rclcpp_construct_ring_buffer : buffer

    rcl_subscription_init ||--|| rclcpp_subscription_init : subscription_handle
    rclcpp_subscription_init ||--|| rclcpp_subscription_callback_added : subscription

    rcl_service_init ||--|| rclcpp_service_callback_added : service_handle
    rclcpp_service_callback_added ||--|| rclcpp_callback_register : callback

    rclcpp_timer_callback_added ||--|| rclcpp_callback_register : callback
    rclcpp_subscription_callback_added ||--|| rclcpp_callback_register : callback

    rclcpp_timer_callback_added ||--|| rcl_timer_init : timer_handle
    rclcpp_timer_link_node ||--|| rcl_timer_init: timer_handle

```

### Tracepoints for representing structure of executor and callback group

A handler such as `timer_handle` or `subscription_handle` is assigned to a callback group. A callback group belongs to an executor.

Relationships of each trace point related to executors are shown as follows.

```mermaid
erDiagram
 construct_executor{
 address executor_addr
 string executor_type_name
 }

 construct_static_executor{
 address executor_addr
 address entities_collector_addr
 string executor_type_name
 }

 add_callback_group{
 address executor_addr
 address callback_group_addr
 string group_type_name
 }

 add_callback_group_static_executor{
 address entities_collector_addr
 address callback_group_addr
 string group_type_name
 }

 callback_group_add_timer{
 address callback_group_addr
 address timer_handle
 }

 callback_group_add_subscription{
 address callback_group_addr
 address subscription_handle
 }

 callback_group_add_service{
 address callback_group_addr
 address service_handle
 }

 callback_group_add_client{
 address callback_group_addr
 address client_handle
 }


 construct_executor ||--o{ add_callback_group : executor_addr
 construct_static_executor ||--o{ add_callback_group_static_executor : entities_collector_addr

    add_callback_group_static_executor ||--o{ callback_group_add_timer : callback_group_addr
    add_callback_group_static_executor ||--o{ callback_group_add_subscription : callback_group_addr
    add_callback_group_static_executor ||--o{ callback_group_add_service : callback_group_addr
    add_callback_group_static_executor ||--o{ callback_group_add_client : callback_group_addr
    add_callback_group ||--o{ callback_group_add_timer : callback_group_addr
    add_callback_group ||--o{ callback_group_add_subscription : callback_group_addr
    add_callback_group ||--o{ callback_group_add_service : callback_group_addr
    add_callback_group ||--o{ callback_group_add_client : callback_group_addr

    callback_group_add_timer ||--|| TIMER_HANDLE : callback_group_addr
    callback_group_add_subscription ||--|| SUBSCRIPTION_HANDLE : callback_group_addr
    callback_group_add_service ||--|| SERVICE_HANDLE : callback_group_addr
    callback_group_add_client ||--|| CLIENT_HANDLE : callback_group_addr


```

### Trace point definition

The following shows the definition of trace points.
A handler such as `timer_handle` or `subscription_handle` is assigned to a callback group. A callback group belongs to an executor.

Trace points with `(caret_trace added)` are hooked and added init_timestamp by caret_trace.
See [Runtime recording](../runtime_processing/runtime_recording.md#tracepoint) for detail.

#### ros2:rcl_init

[Built-in tracepoints]

Sampled items

- void \* context_handle
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rcl_node_init

[Built-in tracepoints]

Sampled items

- void \* node_handle
- void \* rmw_handle
- char \* node_name
- char \* node_namespace
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rcl_publisher_init

[Built-in tracepoints]

Sampled items

- void \* publisher_handle
- void \* node_handle
- void \* rmw_publisher_handle
- char \* topic_name
- size_t queue_depth
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rcl_subscription_init

[Built-in tracepoints]

Sampled items

- void \* subscription_handle
- void \* node_handle
- void \* rmw_subscription_handle
- char \* topic_name
- size_t queue_depth
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_subscription_init

[Built-in tracepoints]

Sampled items

- void \* subscription_handle
- void \* subscription
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_subscription_callback_added

[Built-in tracepoints]

Sampled items

- void \* subscription
- void \* callback
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rcl_service_init

[Built-in tracepoints]

Sampled items

- void \* service_handle
- void \* node_handle
- void \* rmw_service_handle
- char \* service_name

---

#### ros2:rclcpp_service_callback_added

[Built-in tracepoints]

Sampled items

- void \* service_handle
- void \* callback

---

#### ros2:rcl_timer_init

[Built-in tracepoints]

Sampled items

- void \* timer_handle
- int64_t period
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_timer_callback_added

[Built-in tracepoints]

Sampled items

- void \* timer_handle
- void \* callback
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_timer_link_node

[Built-in tracepoints]

Sampled items

- void \* timer_handle
- void \* node_handle
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_callback_register

[Built-in tracepoints]

Sampled items

- void \* callback
- char \* function_symbol
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_ipb_to_subscription (Only for iron or later and intra communication)

[Built-in tracepoints]

Sampled items

- void \* ipb
- void \* subscription
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_buffer_to_ipb (Only for iron or later and intra communication)

[Built-in tracepoints]

Sampled items

- void \* buffer
- void \* ipb
- int64_t init_timestamp (caret_trace added)

---

#### ros2:rclcpp_construct_ring_buffer (Only for iron or later and intra communication)

[Built-in tracepoints]

Sampled items

- void \* buffer
- uint64_t capacity
- int64_t init_timestamp (caret_trace added)

---

#### ros2_caret:caret_init

[Hooked tracepoints]

Sampled items

- int64_t clock_offset
- char \* distribution

---

#### ros2_caret:rmw_implementation

[Hooked tracepoints]

Sampled items

- char \* rmw_impl
- int64_t init_timestamp

---

#### ros2:rcl_client_init

[Built-in tracepoints]

Sampled items

- void \* client_handle
- void \* node_handle
- void \* rmw_client_handle
- char \* service_name

---

#### ros2:rcl_lifecycle_state_machine_init

[Built-in tracepoints]

Sampled items

- void \* node_handle
- void \* state_machine

---

#### ros2_caret:construct_executor

[Hooked tracepoints]

Sampled items

- void \* executor_addr
- char \* executor_type_name
- int64_t init_timestamp

---

#### ros2_caret:construct_static_executor

[Hooked tracepoints]

Sampled items

- void \* executor_addr
- void \* entities_collector_addr
- char \* executor_type_name
- int64_t init_timestamp

---

#### ros2_caret:add_callback_group

[Hooked tracepoints]

Sampled items

- void \* executor_addr
- void \* callback_group_addr
- char \* group_type_name
- int64_t init_timestamp

---

#### ros2_caret:add_callback_group_static_executor

[Hooked tracepoints]

Sampled items

- void \* entities_collector_addr
- void \* callback_group_addr
- char \* group_type_name
- int64_t init_timestamp

---

#### ros2_caret:callback_group_add_timer

[Hooked tracepoints]

Sampled items

- void \* callback_group_addr
- void \* timer_handle
- int64_t init_timestamp

---

#### ros2_caret:callback_group_add_subscription

[Hooked tracepoints]

Sampled items

- void \* callback_group_addr
- void \* subscription_handle
- int64_t init_timestamp

---

#### ros2_caret:callback_group_add_service

[Hooked tracepoints]

Sampled items

- void \* callback_group_addr
- void \* service_handle
- int64_t init_timestamp

---

#### ros2_caret:callback_group_add_client

[Hooked tracepoints]

Sampled items

- void \* callback_group_addr
- void \* client_handle
- int64_t init_timestamp
