# Tracepoints definition

This section lists all tracepoints and their definition.
Before listing tracepoints.

Some tracepoints are used for collecting identification of executors, nodes, callbacks, and topics during application's initialization. They are called initialization tracepoints. The other tracepoints are embedded for sampling timestamps after completion of initialization, and called runtime tracepoints.

Tracepoints are embedded in ROS and DDS layer.
各トレースポイントには、その実装方法を区別するために以下の情報も記載しています。

- Galactic 実装
  - Galactic 以降の ROS にはデフォルトで実装されている、ros2 tracing が使用しているトレースポイントです。
  - service や lifecycle など、実装はされていても CARET では使われていないトレースポイントも有ります。
- CARET フォーク実装
  - rclcpp リポジトリをフォークして追加しているトレースポイントです。
- CARET フック実装
  - LD_PRELOAD によるフックで追加しているトレースポイントです。

> フォークとフックの２つの方法でトレースポイントを追加している理由
>
> CARET は基本的には実装と切り離すために、LD_PRELOAD を利用したフックによるトレースポイントの追加をしています。  
> ただし、rclcpp には、プロセス内通信などのテンプレートによる実装を含み、LD_PRELOAD ではフックできない処理があります。  
> LD_PRELOAD は実行時の動的リンクに対しフックできる機能であるのに対し、  
> テンプレートで実装された処理はコンパイルでバイナリ内に組み込まれてしまうためフックできません。  
> このようなテンプレートで実装された処理に対してのみ、rclcpp をフォークしてトレースポイントを追加しています。  
> このような理由から、測定対象のアプリケーションを再ビルドする必要が生じています。

## Sequence diagram of major tracepoints

```plantuml
@startuml
title: Definition of major trace points

participant UserCode
participant ROS2
participant DDS
participant LTTng

== Publisher Side ==

activate ROS2
activate UserCode
UserCode -> ROS2: publish()
activate ROS2
ROS2 --> LTTng: [rclcpp_publish] \n[rclcpp_intra_publish]


ROS2 -> DDS: dds_write()
deactivate ROS2
activate DDS
DDS --> LTTng: [dds_write]

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
    ROS2 --> LTTng : [on_data_available]
    deactivate ROS2

    deactivate DDS
    ROS2 -> DDS : reset ready-set, take messages
    activate DDS
    DDS -> ROS2
    deactivate DDS

    group execute timer callbacks
    end

    group execute subscription callbacks
        ROS2 --> LTTng: [callback_start]
        ROS2 -> UserCode: callback start
        activate UserCode
        UserCode -> ROS2: callback end
        deactivate UserCode
        ROS2 --> LTTng: [callback_end]
    end

    group execute service callbacks
    end

    group execute client callbacks
    end
end

deactivate ROS2
@enduml
```

## Initialization tracepoints

---

### ros2:rcl_init

[Galactic 実装]

付加情報

- void \* context_handle

---

### ros2:rcl_node_init

[Galactic 実装]

付加情報

- void \* node_handle
- void \* rmw_handle
- char \* node_name
- char \* node_namespace

---

### ros2:rcl_publisher_init

[Galactic 実装]

付加情報

- void \* publisher_handle
- void \* node_handle
- void \* rmw_publisher_handle
- char \* topic_name
- size_t queue_depth

---

### ros2:rcl_subscription_init

[Galactic 実装]

付加情報

- void \* subscription_handle
- void \* node_handle
- void \* rmw_subscription_handle
- char \* topic_name
- size_t queue_depth

---

### ros2:rclcpp_subscription_init

[Galactic 実装]

付加情報

- void \* subscription_handle
- void \* subscription

---

### ros2:rclcpp_subscription_callback_added

[Galactic 実装]

付加情報

- void \* subscription
- void \* callback

---

### ros2:rcl_timer_init

[Galactic 実装]

付加情報

- void \* timer_handle
- int64_t period

---

### ros2:rclcpp_timer_callback_added

[Galactic 実装]

付加情報

- void \* timer_handle
- void \* callback

---

### ros2:rclcpp_timer_link_node

[Galactic 実装]

付加情報

- void \* timer_handle
- void \* node_handle

---

### ros2:rclcpp_callback_register

[Galactic 実装]

付加情報

- void \* callback
- char \* function_symbol

---

### ros2_caret:rmw_implementation

[CARET フック実装]

付加情報

- char \* rmw_impl

---

### ros2_caret:construct_executor

[CARET フック実装]

付加情報

- void \* executor_addr
- char \* executor_type_name

---

### ros2_caret:construct_static_executor

[CARET フック実装]

付加情報

- void \* executor_addr
- void \* entities_collector_addr
- char \* executor_type_name

---

### ros2_caret:add_callback_group

[CARET フック実装]

付加情報

- void \* executor_addr
- void \* callback_group_addr
- char \* group_type_name

---

### ros2_caret:add_callback_group_static_executor

[CARET フック実装]

付加情報

- void \* entities_collector_addr
- void \* callback_group_addr
- char \* group_type_name

---

### ros2_caret:callback_group_add_timer

[CARET フック実装]

付加情報

- void \* callback_group_addr
- void \* timer_handle

---

### ros2_caret:callback_group_add_subscription

[CARET フック実装]

付加情報

- void \* callback_group_addr
- void \* subscription_handle

---

### ros2_caret:callback_group_add_service

[CARET フック実装]

付加情報

- void \* callback_group_addr
- void \* service_handle

---

### ros2_caret:callback_group_add_client

[CARET フック実装]

付加情報

- void \* callback_group_addr
- void \* client_handle

---

## ランタイム時のトレースポイント

### ros2:callback_start

[Galactic 実装]

付加情報

- void \* callback
- bool is_intra_process

---

### ros2:callback_end

[Galactic 実装]

付加情報

- void \* callback

---

### ros2:message_construct

[CARET フォーク実装]

付加情報

- void \* original_message
- void \* constructed_message

---

### ros2:rclcpp_intra_publish

[CARET フォーク実装]

付加情報

- void \* publisher_handle
- void \* message
- uint64_t message_timestamp

---

### ros2:dispatch_subscription_callback

[CARET フォーク実装]

付加情報

- void \* message
- void \* callback
- uint64_t source_timestamp
- uint64_t message_timestamp

---

### ros2:dispatch_intra_process_subscription_callback

[CARET フォーク実装]

付加情報

- void \* message
- void \* callback
- uint64_t message_timestamp

---

### ros2:rcl_publish

[Galactic 実装]

付加情報

- void \* publisher_handle
- void \* message

---

### ros2:rclcpp_publish

[Galactic 実装]

付加情報

- void \* publisher_handle
- void \* message
- uint64_t message_timestamp

### ros2_caret:dds_write

[CARET フック実装]

付加情報

- void \* message

---

### ros2_caret:dds_bind_addr_to_stamp

[CARET フック実装]

付加情報

- void \* addr
- uint64_t source_stamp

---

### ros2_caret:dds_bind_addr_to_addr

[CARET フック実装]

付加情報

- void \* addr_from
- void \* addr_to

---

## トレースポイントの紐付け

本ページで列挙しているトレースポイントはノード ID（`node_handle`）やコールバック ID（`コールバックのインスタンスのアドレス`）などの値を元に紐付けて利用する。  
それぞれのトレースポイントの対応関係をグラフで示す。

### ノードの初期化

```mermaid

graph RL
  node_handle[[node_handle]]
  subscription_handle[[subscription_handle]]
  subscription[[subscription]]
  callback[[callback]]
  timer_handle[[timer_handle]]
  client_handle[[client_handle]]
  node_handle[[node_handle]]
  publisher_handle[[publisher_handle]]

  rcl_init[rcl_init<ul><li>context_handle</li></ul>]

  rcl_node_init[rcl_node_init<br /><ul><li>node_handle</li><li>node_name</li><li>node_namespace</li><li>rmw_handle</li></ul>]

  rcl_publisher_init[rcl_publisher_init<br /><ul><li>node_handle</li><li>publisher_handle</li><li>rmw_publisher_handle</li><li>topic_name</li><li>queue_depth</li></ul>]

  rcl_subscription_init[rcl_subscription_init<ul><li>subscription_handle</li><li>node_handle</li><li>rmw_subscription_handle</li><li>topic_name</li><li>queue_depth</li></ul>]

  rclcpp_subscription_init[rclcpp_subscription_init<ul><li>subscription_handle</li><li>subscription</li></ul>]

  rclcpp_subscription_callback_added[rclcpp_subscription_callback_added<ul><li>subscription</li><li>callback</li></ul>]

  rclcpp_timer_link_node[rclcpp_timer_link_node<ul><li>timer_handle</li><li>node_handle</li></ul>]

  rclcpp_callback_register[rclcpp_callback_register<ul><li>callback</li><li>function_symbol</li></ul>]

  rmw_implementation[rmw_implementation<ul><li>rmw_impl</li></ul>]

  rcl_timer_init[rcl_timer_init<ul><li>timer_handle</li><li>period</li></ul>]


  rcl_publisher_init <--> node_handle
  node_handle <--> rcl_node_init
  rcl_subscription_init <--> node_handle
  subscription_handle <--> rcl_subscription_init
  rclcpp_subscription_init <--> subscription_handle

  publisher_handle <--> rcl_publisher_init

  subscription <--> rclcpp_subscription_init
  rclcpp_subscription_callback_added <--> subscription
  callback <--> rclcpp_subscription_callback_added

  rcl_timer_init <--> timer_handle
  rclcpp_timer_callback_added <--> timer_handle
  callback <-->  rclcpp_timer_callback_added
  timer_handle <--> rclcpp_timer_link_node
  rclcpp_timer_link_node <--> node_handle

  callback <--> rclcpp_callback_register
  rmw_implementation

```

コールバックのインスタンスのアドレス値が一意に決まれば、`timer_callback_added`など他のトレースポイントの値を紐付いていくことでノードの情報まで得ることができる。  
逆に、node_handle が一意に決まれば、そのノードに含まれるコールバックも特定できる。

### エグゼキューターやコールバックグループの初期化

```mermaid

graph RL
  subscription_handle[[subscription_handle]]
  executor_addr[[executor_addr]]
  entities_collector_addr[[entities_collector_addr]]
  timer_handle[[timer_handle]]
  client_handle[[client_handle]]
  service_handle[[service_handle]]
  callback_group_addr[[callback_group_addr]]

  construct_executor[construct_executor<ul><li>executor_addr</li><li>executor_type_name</li></ul>]

  construct_static_executor[construct_static_executor<ul><li>executor_addr</li><li>executor_type_name</li><li>entities_collector_addr</li></ul>]

  add_callback_group[add_callback_group<ul><li>executor_addr</li><li>callback_group_addr</li><li>group_type_name</li></ul>]

  add_callback_group_static_executor[add_callback_group_static_executor<ul><li>entities_collector_addr</li><li>callback_group_addr</li><li>group_type_name</li></ul>]

  callback_group_add_timer[callback_group_add_timer<ul><li>callback_group_addr</li><li>timer_handle</li></ul>]

  callback_group_add_subscription[callback_group_add_subscription<ul><li>callback_group_addr</li><li>subscription_handle</li></ul>]

  callback_group_add_service[callback_group_add_service<ul><li>callback_group_addr</li><li>service_handle</li></ul>]


  callback_group_add_client[callback_group_add_client<ul><li>callback_group_addr</li><li>client_handle</li></ul>]

  executor_addr <--> construct_executor
  entities_collector_addr <--> construct_static_executor
  add_callback_group <--> executor_addr
  callback_group_addr <--> add_callback_group

  add_callback_group_static_executor <--> entities_collector_addr
  callback_group_addr <--> add_callback_group_static_executor

  callback_group_add_timer <--> callback_group_addr
  timer_handle <--> callback_group_add_timer

  callback_group_add_subscription <--> callback_group_addr
  subscription_handle <--> callback_group_add_subscription

  callback_group_add_service <--> callback_group_addr
  service_handle <--> callback_group_add_service

  callback_group_add_client <--> callback_group_addr
  client_handle <--> callback_group_add_client
```

タイマーなどの各ハンドラはコールバックグループに追加され、エグゼキュータに紐付けられる。

### メッセージの publish まで

```mermaid

graph LR
  source_timestamp[[source_timestamp]]
  message_dds[[message_addr]]
  message_intra[[message_addr]]
  message_inter[[message_addr]]
  publish(("publish(message)"))
  source_timestamp[[source_timestamp]]
  message_intra_sub[[message_addr]]

  rclcpp_intra_publish[rclcpp_intra_publish<ul><li>publisher_handle</li><li>message_addr</li><li>message_timestamp</li></ul>]

  message_construct_intra[message_construct<ul><li>original_message_addr</li><li>constructed_message_addr</li></ul>]
  message_construct_inter[message_construct<ul><li>original_message_addr</li><li>constructed_message_addr</li></ul>]

  rclcpp_publish[rclcpp_publish<ul><li>publisher_handle</li><li>message_addr</li><li>message_timestamp</li></ul>]

  rcl_publish[rcl_publish<ul><li>publisher_handle</li><li>message_addr</li></ul>]
  dds_write[dds_write<ul><li>message_addr</li></ul>]
  dds_bind_addr_to_addr[dds_bind_addr_to_addr<ul><li>addr_from</li><li>addr_to</li></ul>]
  dds_bind_addr_to_stamp[dds_bind_addr_to_stamp<ul><li>message_addr</li><li>source_timestamp</li></ul>]

  publish -->  rclcpp_intra_publish
  publish -->  rclcpp_publish

  subgraph message_copy_if_necessary
    rclcpp_intra_publish <--> message_intra
    rclcpp_publish <--> message_inter
    message_construct_intra -.-> message_intra
    message_intra -.-> message_construct_intra
    message_construct_inter -.-> message_inter
    message_inter -.-> message_construct_inter
  end


  message_inter <--> rcl_publish
  message_inter <--> dds_write
  dds_write <--> message_dds

  message_dds <-->  dds_bind_addr_to_stamp
  dds_bind_addr_to_stamp <--> source_timestamp

  dds_bind_addr_to_addr -.-> message_dds
  message_dds -.-> dds_bind_addr_to_addr

```

`publish`を実行した際、unique_ptr 型で複数の subscription がある場合などのケースでは、必要に応じてメッセージのコピーが発生する。  
メッセージのコピーが発生した際は`message_construct`でコピー前とコピー後の変数のアドレスを紐付けられるようにしている。  
rcl レイヤー以下ではメッセージのアドレスで対応付けられるようにし、DDS レイヤーで source_timestamp に紐付けられるようにしている。  
source_timestamp は元々は QoS の Deadline などに使われる値で、全ての DDS 通信を行うメッセージに自動的に付与される値であり、subscription 側で同じ値が受信される。  
受信側では source_timestamp の値を紐付けるための情報として利用している。

message_addr（プロセス内通信）/source_timestamp（プロセス間通信）から publish までが一意に紐付けられる。

### メッセージの受信からコールバック実行まで

```mermaid

graph LR
  source_timestamp[[source_timestamp]]
  message_intra_sub[[message_addr]]
  message_intra[[message_addr]]
  callback[[callback]]
  source_timestamp[[source_timestamp]]
  source_timestamp_sub[source_timestamp]

  dispatch_subscription_callback[dispatch_subscription_callback<ul><li>message_addr</li><li>callback</li><li>source_timestamp</li><li>message_timestamp</li></ul>]
  dispatch_intra_process_subscription_callback[dispatch_intra_process_subscription_callback<ul><li>message_addr</li><li>callback</li><li>message_timestamp</li></ul>]

  callback_end_[callback_end<ul><li>callback</li></ul>]
  callback_start[callback_start<ul><li>callback</li><li>is_intra_process</li></ul>]


  source_timestamp --Inter Process Communication--> source_timestamp_sub
  message_intra --Intra Process Communication--> message_intra_sub
  message_intra_sub --> dispatch_intra_process_subscription_callback

  dispatch_intra_process_subscription_callback --> callback

  callback --> callback_start
  callback --> callback_end_

  dispatch_subscription_callback --> callback
  source_timestamp_sub --> dispatch_subscription_callback
```

プロセス内通信では publish されたメッセージのアドレスで紐づけを行う。  
プロセス間通信では source_timestamp で紐づけを行う。

callback_start から message_addr/source_timestamp までが一意に紐付けられる。  
また、publisher 側のグラフで示した通り、message_addr/source_timestamp から publish までが一意に紐付けられる。  
したがって、publish から callback_start までは一意に紐付けることができる。

ただし、コールバックの実行と publish の紐付けはできていないので注意。
