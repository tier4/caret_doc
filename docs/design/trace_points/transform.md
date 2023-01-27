# Transform 対応

## 基本的な挙動

```plantuml
broadcaster@process1 --> listener@process2 : /tf に publish
listener_buffer@process2 --> listener@process2 : TF バッファの更新
=====
listener_app@process2 --> listener@process2 : lookupTransform
listener_buffer@process2 --> listener@process2 : バッファを確認。
listener_buffer@process2 --> listener@process2 : 指定された時刻の座標を算出
listener_buffer@process2 --> listener@process2 : TF Treeを辿って、座標変換の算出
listener_buffer@process2 --> listener_app@process2 : 座標返却／エラー
```

### Broadcaster側

- broadcaster側は、sendTransformを実行したスレッドがそのままpublishしている
- TransformBroadcasterのコンストラクタに与えたノードで 新規にpublisherが作られる([該当コード](https://github.com/ros2/geometry2/blob/de08d177d424f0d73efd4ff5951fd1eaabc9638d/tf2_ros/include/tf2_ros/transform_broadcaster.h#L61))
- sendTransformに入力するメッセージの型はgeometry_msgs::msg::TransformStamped。
- 内部では、tf2_msgs::msg::TFMessageへ変換して/tfトピックへpublishしている。

```
#geometry_msgs/TransformStamped.msg
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
```

```
#tf2_msgs::msg::TFMessage
geometry_msgs/TransformStamped[] transforms
```

### listener側

最新の座標データを保持した座標データのバッファを介して以下が行われる

- /tfトピックから受け取ったメッセージの保存
  （常にバッファを最新にさせるため、別スレッドで処理を行わせるのが基本。）

- チュートリアル通りにtf2*ros::TransformListener(buf) と使った場合：
  /tfのsubscribe、バッファの更新は非同期に行われる。（非同期に行いたかったという旨が論文にかかれている）
  新規スレッド＆新規エグゼキュータ＆新規ノードが作成させる
  Listener毎に transform_listener_impl*\*\* 名前を持ったノードが作成される（だから量産されてる）
  [対応するコンストラクタ](https://github.com/ros2/geometry2/blob/210791c2a57a5c7c87e348e684c9cdc28469cb70/tf2_ros/include/tf2_ros/transform_listener.h#L87)

新規スレッドを立ち上げさせず、アプリケーション側で制御する方法もある。（動作未確認）

ノードも引数に与え、こっちのコンストラクタでListenerを構築すること。
スレッドやエグゼキュータの生成も行われず、引数に与えたノードで/tfのsubscribe＆バッファの更新が行われる

<https://github.com/ros2/geometry2/blob/210791c2a57a5c7c87e348e684c9cdc28469cb70/tf2_ros/include/tf2_ros/transform_listener.h#L89>

lookupTransformなどを受け付け、指定された時刻の座標変換を取得
（こちらはアプリケーション側が呼ぶAPI）

デフォルトだと、10秒前まで座標を遡れるようになっている。
<https://github.com/ros2/geometry2/blob/16562cee8694c11f1f82b2bdbde2814fca1c7954/tf2/include/tf2/buffer_core.h#L70>

## tfの動作シーケンス図

トレースポイントの挿入箇所
<https://github.com/ros2/geometry2>

※ シーケンス図は説明のために簡略化しています。

```plantuml
participant SendNode
participant TransformBroadcaster
participant TransformListenerNode
participant Buffer
participant LookupNode
participant Lttng

activate LookupNode
LookupNode-> Buffer : lookupTransform
activate Buffer
Buffer ->> Lttng : lookup transform start
activate SendNode
SendNode -> TransformBroadcaster : sendTransform(&transform)
activate TransformBroadcaster
TransformBroadcaster ->> Lttng : sendtransform
TransformBroadcaster ->> TransformListenerNode : publish & subscribe
activate TransformListenerNode
TransformBroadcaster --> SendNode
deactivate SendNode

deactivate TransformBroadcaster
TransformListenerNode -> Buffer
activate Buffer
Buffer ->> Lttng : set transform
Buffer --> TransformListenerNode
deactivate Buffer
deactivate TransformListenerNode
Buffer -> Buffer : find closest \n interpolate
Buffer ->> Lttng : lookup transform end
Buffer --> LookupNode : return transform
deactivate Buffer
```

---

TransformBroadcasterがrclcpp::publishするまで

```plantuml
participant TransformBroadcaster
participant Publisher
participant Lttng

-> TransformBroadcaster : sendTransform(&transform)
activate TransformBroadcaster
TransformBroadcaster -> TransformBroadcaster : convert To transforms
-> TransformBroadcaster : sendTransform(&transforms)
TransformBroadcaster ->> Lttng : send_transform
TransformBroadcaster -> TransformBroadcaster : vector<TransformStamped> to TFMessage
TransformBroadcaster -> Publisher : publish TFMessage
activate Publisher
Publisher -> : publish message
deactivate Publisher
deactivate TransformBroadcaster

```

---

メッセージ受信からBufferに格納するまで

```plantuml
participant TransformListener
participant Buffer
participant TimeCache
participant Lttng

==Bufferに格納==
-> TransformListener : subscribe TFMessage
activate TransformListener

loop
TransformListener -> Buffer : setTransform(TransformStamped)
activate Buffer
Buffer ->> Lttng: set_transform
Buffer -> TimeCache : insertData(TransformStorage)
activate TimeCache
TimeCache --> Buffer
deactivate TimeCache
Buffer --> Buffer : testTransformableRequests \n [execute added TransformableCallback if possible]
Buffer --> TransformListener
deactivate Buffer
end
deactivate TransformListener
```

---

lookupTransformを実行してBufferから取得するまで

```plantuml
participant TransformListener
participant Buffer
participant BufferCore
participant TransformAccum
participant Lttng

-> Buffer : lookupTransform()
activate Buffer
Buffer ->> Lttng: lookup_transform
Buffer -> Buffer : canTransform\n[wait until available or timeout]
Buffer -> BufferCore : lookupTransform
activate BufferCore
BufferCore -> BufferCore : walkToTopParent(TransformAccum &f)
activate BufferCore
loop
BufferCore -> BufferCore : getFrame
BufferCore -> TransformAccum : f.gather(frame)
activate TransformAccum
TransformAccum --> BufferCore
deactivate TransformAccum
BufferCore -> TransformAccum : f.accum()
activate TransformAccum
TransformAccum --> BufferCore
deactivate TransformAccum
end
deactivate BufferCore
BufferCore --> Buffer
deactivate BufferCore
<-- Buffer
deactivate Buffer

```

---

waitForTransformを実行

```plantuml
participant Buffer
participant BufferCore

==setCreateTimer==
-> CreateTimerROS : const CreateTimerROS(NodeBase, NodeTimer)
activate CreateTimerROS
<-- CreateTimerROS : CreateTimerROS
deactivate CreateTimerROS

-> Buffer : setCreateTimerInterface(CreateTimer)
activate Buffer
<-- Buffer
deactivate Buffer

==waitForTransform==

-> Buffer : waitForTransform(TransformReadyCallback)
activate Buffer
Buffer -> BufferCore : addTransformableRequest(TransformableCallback)
activate BufferCore
BufferCore --> Buffer : request_handle
deactivate BufferCore

alt if handle == 0
    Buffer -> Buffer : lookupTransform()
    Buffer -> Buffer : promise.set_value()
    Buffer -> TransformReadyCallback : callback(future)\n[execute immediately]
    activate TransformReadyCallback
    TransformReadyCallback --> Buffer
    deactivate TransformReadyCallback

else if handle == 0xfff...f
    Buffer -> Buffer : promise.set_exception
    Buffer -> TransformReadyCallback : callback(future)\n[execute immediately]
    activate TransformReadyCallback
    TransformReadyCallback --> Buffer
    deactivate TransformReadyCallback

else else
    Buffer -> CreateTimerROS : createTimer()
    activate CreateTimerROS
    CreateTimerROS --> Buffer : Timer
    deactivate CreateTimerROS
    Buffer -> Buffer : request timer callback for timeout
end


<-- Buffer : TransformStampedFuture

deactivate Buffer
```

---

TransformStampedFuture使用ケース

中身はC++のFuture。そのまま使うことができる。
その他、[rclcpp.spin_until_future_complete()](https://docs.ros2.org/galactic/api/rclcpp/namespacerclcpp.html#ab83b41b70748bbd4631b498596148360)なども使用できる。

```cpp
using TransformStampedFuture =
    std::shared_future<geometry_msgs::msg::TransformStamped>;

using TransformReadyCallback =
    std::function<void (const TransformStampedFuture &)>;
```

```plantuml

-> TransformStampedFuture : wait_for()
-> TransformStampedFuture : wait_until()
-> TransformStampedFuture : get()
[<--TransformStampedFuture
```

---

waitForTransformの引数であるTransformReadyCallbackを使用するケース

```plantuml
participant Buffer
participant BufferCore
participant TransformReadyCallback

==Timeoutケース==
]-> Buffer : execute timer callback
activate Buffer
Buffer -> Buffer : remove timer request
Buffer -> Buffer : promise.set_exception()
Buffer -> TransformReadyCallback : callback(future)
activate TransformReadyCallback
TransformReadyCallback --> Buffer
deactivate TransformReadyCallback
]<-- Buffer
deactivate Buffer

==Availableケース==
]-> BufferCore  : testTransformableRequests
activate BufferCore
BufferCore -> Buffer : execute TransformableCallback()
activate Buffer
    Buffer -> Buffer : remove timer callback for timeout
    alt if timeout not occured and TransformAvailable
        Buffer -> Buffer : lookupTransform()
        Buffer -> Buffer : promise.set_value()
        Buffer -> TransformReadyCallback : callback(future)
        activate TransformReadyCallback
        TransformReadyCallback --> Buffer
        deactivate TransformReadyCallback
    else if timeout not occured and not Availavle
        Buffer -> Buffer : promise.set_exception()
        Buffer -> TransformReadyCallback : callback(future)
        activate TransformReadyCallback
        TransformReadyCallback --> Buffer
        deactivate TransformReadyCallback
    end
Buffer --> BufferCore
deactivate Buffer
BufferCore-->
deactivate BufferCore

```
