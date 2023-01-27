### Relationships of each runtime trace points

```mermaid
erDiagram

 callback_start{
 address callback
 bool is_intra_process
 }

 callback_end{
 address callback
 }

 message_construct{
 address original_message
 address constructed_message
 }

 dds_bind_addr_to_addr{
 address addr_from
 address addr_to
 }

 rclcpp_intra_publish{
 address publisher_handle
 address message
 uint64_t message_timestamp
 }

 dispatch_subscription_callback{
 address message
 address callback
 uint64_t source_timestamp
 uint64_t message_timestamp
 }

 dispatch_intra_process_subscription_callback{
 address message
 address callback
 uint64_t message_timestamp
 }

 rcl_publish{
 address publisher_handle
 address message
 }

 rclcpp_publish{
 address publisher_handle
 address message
 uint64_t message_timestamp
 }

 dds_write{
 address message
 }

 dds_bind_addr_to_stamp{
 address addr
 uint64_t source_stamp
 }

    rclcpp_intra_publish ||--|| dispatch_intra_process_subscription_callback: message_addr
    rclcpp_publish ||--|| rcl_publish: message_addr
    rcl_publish ||--|| dds_write: message_addr
    dds_write ||--|| dds_bind_addr_to_stamp: message_addr

    dds_bind_addr_to_stamp ||--|| dispatch_subscription_callback: source_timestamp


    dispatch_intra_process_subscription_callback ||--|| callback_start: callback
    dispatch_subscription_callback ||--|| callback_start: callback
    callback_start ||--|| callback_end: callback

```

Using addresses, thread id (`tid`) and source timestamp, CARET is able to identify a pair of message publish and corresponding subscription.
However, it's difficult to associate a certain message publish to corresponding callback execution because mapping between callback and publish cannot be obtained automatically.

`message_construct` and `dds_bind_addr_to_addr` are trace points to adapt to copying and converting instances for binding.

```mermaid
erDiagram
    send_transform {
        address tf_broadcaster
        uint64_t[] stamps
        uint32_t[] frame_ids_compact
        uint32_t[] child_frame_ids_compact
    }
    init_tf_broadcaster_frame_id_compact {
        address tf_broadcaster
        string frame_id
        uint32_t frame_id_compact
    }

    init_tf_buffer_frame_id_compact {
        address tf_buffer_core
        string frame_id
        uint32_t frame_id_compact
    }

    tf_lookup_transform {
        address tf_buffer_core
        uint32_t target_frame_id_compact
        uint32_t source_frame_id_compact
    }
    tf_buffer_find_closest{
        address tf_buffer_core
        uint32_t frame_id_compact
        uint32_t child_frame_id_compact
        uint64_t stamp
        uint32_t frame_id_compact_
        uint32_t child_frame_id_compact_
        uint64_t stamp_
    }
    tf_set_transform{
        address tf_buffer_core
        uint64_t stamp
        uint32_t frame_id_compact
        uint32_t child_frame_id_compact
    }

    send_transform ||--o| init_tf_broadcaster_frame_id_compact: frame_id_compact__to__frame_id
    send_transform ||--|{ tf_set_transform: frame_id__and__child_frame_id__and_stamp
    tf_set_transform }|--|{ tf_buffer_find_closest: frame_id_compact__and_frame_id_compact__and__stamp
    tf_buffer_find_closest }|--|| tf_lookup_transform: todo
    tf_set_transform ||--|| rclcpp_publish: tid
    init_tf_buffer_frame_id_compact |o--|| tf_lookup_transform: todo
```

### Trace point definition

#### ros2:callback_start

[Built-in tracepoints]

Sampled items

- void \* callback
- bool is_intra_process

---

#### ros2:callback_end

[Built-in tracepoints]

Sampled items

- void \* callback

---

#### ros2:message_construct

[Extended tracepoints]

Sampled items

- void \* original_message
- void \* constructed_message

---

#### ros2:rclcpp_intra_publish

[Extended tracepoints]

Sampled items

- void \* publisher_handle
- void \* message
- uint64_t message_timestamp

---

#### ros2:dispatch_subscription_callback

[Extended tracepoints]

Sampled items

- void \* message
- void \* callback
- uint64_t source_timestamp
- uint64_t message_timestamp

---

#### ros2:dispatch_intra_process_subscription_callback

[Extended tracepoints]

Sampled items

- void \* message
- void \* callback
- uint64_t message_timestamp

---

#### ros2:rcl_publish

[Built-in tracepoints]

Sampled items

- void \* publisher_handle
- void \* message

---

#### ros2:rclcpp_publish

[Built-in tracepoints]

Sampled items

- void \* publisher_handle
- void \* message
- uint64_t message_timestamp

#### ros2_caret:dds_write

[Hooked tracepoints]

Sampled items

- void \* message

---

#### ros2_caret:dds_bind_addr_to_stamp

[Hooked tracepoints]

Sampled items

- void \* addr
- uint64_t source_stamp

---

#### ros2_caret:dds_bind_addr_to_addr

[Hooked tracepoints]

Sampled items

- void \* addr_from
- void \* addr_to

#### ros2_caret:send_transform

[Hooked tracepoints]

Sampled items

- void \* tf_broadcaster
- uint64_t[] stamps
- uint32_t[] frame_ids_compact
- uint32_t[] child_frame_ids_compact

#### ros2_caret:init_tf_broadcaster_frame_id_compact

[Hooked tracepoints]

Sampled items

- void \* tf_broadcaster
- char \* frame_id
- uint32 frame_id_compact

#### lookup_transform_start

- void \* tf_buffer_core
- uint32_t target_frame_id_compact
- uint32_t source_frame_id_compact

#### tf_buffer_find_closest

- void \* tf_buffer_core
- uint32_t frame_id_compact
- uint32_t child_frame_id_compact
- uint64_t stamp
- uint32*t frame_id_compact*
- uint32*t child_frame_id_compact*
- uint64*t stamp*

#### tf_set_transform

- void \* tf_buffer_core
- uint64_t stamp
- uint32_t frame_id_compact
- uint32_t child_frame_id_compact

#### init_tf_buffer_frame_id_compact

- void \* tf_buffer_core
- char \* frame_id
- uint32_t frame_id_compact
