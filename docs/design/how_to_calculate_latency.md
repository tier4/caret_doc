# レイテンシの計算方法

## 通信レイテンシとノードレイテンシの算出方法
以下は通信レイテンシとノードレイテンシを表した図です。

![callback_and_node_latency](../imgs/callback_and_node_latency.png)

ここでは通信レイテンシとノードレイテンシの算出方法について説明します。

### 通信レイテンシの算出方法
通信レイテンシはコールバックのpublishの時刻と、後続のコールバックが開始した時刻の差分で計算しています。

#### プロセス内通信
次の表はプロセス内通信のトレースに必要なトレースポイントと、そのトレースポイントの引数（トレースデータとして出力されるもの）です。※簡易版


| トレースポイント名 | 引数1 | 引数2 | 時刻 |
|-|-|-|-|
| rclcpp_intra_publish | publisher_handle_arg | <span style="color: red; ">message_arg</span> | time1 |
| dispatch_intra_process_subscription_callback | <span style="color: red; ">message_arg</span> | <span style="color: green; ">callback_arg</span> | time2 |
| callback_start | <span style="color: green; ">callback_arg</span> | is_intra_process | time3 |

※[引数についてはこちらを参照](https://tier4.github.io/CARET_doc/design/tracepoint_definition/)

message_argとcallback_argにはメッセージのアドレス、コールバックのアドレスが格納されています。
rclcpp_intra_publisherとdispatch_intra_process_subscription_callbackは同じmessage_argを持つトレースポイント同士で紐づけ、dispatch_intra_process_subscription_callbackとcallback_startは同じcallback_argを持つトレースポイント同士で紐づけることにより、下記のような表を作成します。
最後に`callback_start - rclcpp_intra_publish`で**プロセス内通信のレイテンシ**を算出しています。

|idx| rclcpp_intra_publish | dispatch_intra_process_subscription_callback | callback_start |
|-|-|-|-|
|0| time1 | time2 | time3 |
|1| ... | ... | ... |


#### プロセス間通信
以下はプロセス間通信のトレースポイントとその引数です。

| トレースポイント名 | 引数1 | 引数2 | 引数3 | 時刻 |
|-|-|-|-|-|
| rclcpp_publish | publisher_handle_arg | <span style="color: red; ">message_arg</span>@1   |   | time1 |
| rcl_publish | publisher_handle_arg | <span style="color: red; ">message_arg</span>@1   |   | time2 |
| dds_write | publisher_handle_arg | <span style="color: red; ">message_arg</span>@1   |   | time3 |
| dds_bind_addr_to_stamp | <span style="color: red; ">message_arg</span>@1 | <span style="color: green; ">stamp_arg</span> |  | time4 |
| dispatch_subscription_callback | messsage_arg@2 | <span style="color: green; ">stamp_arg</span> | <span style="color: blue; ">callback_arg</span> | time5 |
| callback_start | <span style="color: blue; ">callback_arg</span> | is_intra_process |  | time6 |


プロセス内通信と同様に同じ引数を持つトレースポイント同士を紐づけていき、publishからcallback_startまでの表を作成します（下記表）。  
1行が1つのプロセス間通信のチェーンを表し、`callback_start - rclcpp_publish` にて**プロセス間通信のレイテンシ**を算出します。

| idx | rclcpp_publish | rcl_publish | dds_write | dds_bind_addr_to_stamp | dispatch_subscription_callback | callback_start |
|-|-|-|-|-|-|-|
| 0 | time1 | time2 | time3 | time4 | time5 | time6 |
| 1 | ... | ... | ... | ... | ... | ...　|


## ノードレイテンシの算出方法
ノードレイテンシの算出方法は2つあります。
それぞれの手法・長所・短所を説明します。

### コールバックチェーンの利用
コールバックレイテンシの測定は、同じコールバックアドレスを持つcallback_startとrclcpp_publishの差分を使って算出します。  
callback_startとrclcpp_publishの紐づけは、rclcpp_publishから見て一番近いcallback_startと紐づけます。

| idx | callback_start (cb_A) [s] | rclcpp_publish (cb_A) [s] | callback_end (cb_A) [s] | callback_arg (cb_A) |
|-|-|-|-|-|
| 0 | 0 | 3 | 4 | 0x1000 |
| 1 | 2 | 5 | 6 | 0x1000 |
| 2 | 4 | 7 | 8 | 0x1000 |
| ... | ... | ... | ... | ... |

| idx | callback_start (cb_B) [s] | rclcpp_publish (cb_B) [s] | callback_end (cb_B) [s] | callback_arg (cb_B) |
|-|-|-|-|-|
| 0 | 4 | 8 | 9 | 0x2000 |
| 1 | 8 | 12 | 13 | 0x2000 |
| 2 | 10 | 14 | 15 | 0x2000 |

上記表のようにコールバックA・B（cb_A・cb_B）が存在し、A→Bと処理が続く時、cb_Aのcallback_endとcb_Bのcallback_startを結び付けて表を作ります。  
最後のcallbackだけはpublishの時の時刻を採用し、下記表のように一つのテーブルにします。

| idx | callback_start (cb_A) [s] | callback_end (cb_A) [s] | callback_start (cb_B) [s] | rclcpp_publish (cb_B) [s] |
|-|-|-|-|-|
| 0 | 0 | 4 | 4 | 8 |
| 1 | 2 | 6 | Lost | Lost |
| 2 | 4 | 8 | 8 | 12 |
| ... | ... | ... | ... | ... |

上記のようにコールバックチェーンをつなぎ、```rclcpp_publish (cb_X) - callback_start (cb_Y)```で**ノードレイテンシ**を算出します。

> ※ノード内にコールバックが１つの場合、X, Yは同じものを指します。  
> 複数コールバックがある場合は、Xが最後・Yが最初のコールバックを指します。




## トレースポイントの挿入箇所

CARETで使われているトレースポイントはROS 2 Galacticで追加したものとCARETで追加したものの2種類あります。
CARETで追加したものは直接書き換えているものとLD_PRELOADによるフックで追加しているものの2種類あります。

![tracepoints_location](../imgs/tracepoints_location.png)


図上で緑色の枠で示されている部分は関数を直接書き換えてトレースポイントを追加しています。
水色の枠で示されている部分はLD_PRELOADを用いたフックによって追加しています。

それぞれの方法に長所・短所があります。

直接書き換える方法ではどの部分でも書き換えられるという長所があります。

フックの方法はメンテナンス性が向上するという長所があります。

rclcpp層のheaderはフックによる方法で追加できなかったため直接書き換えています。


## トレースポイントの読み込まれ方

それぞれの方法で追加したトレースポイントがどのように読み込まれるか説明します。

![tracepoints_call](../imgs/tracepoints_call.png)

緑色の枠は直接書き換えている場合です。
関数内に直接トレースポイントの関数があります。


水色の枠はフックによる方法です。
フックによる方法では自分で定義した関数が元の関数より優先的に読み込まれるようになっています。
.soファイル内にトレースポイントの関数と、元の関数を読み込む関数を記載します。
元の関数を呼ぶ処理によってトレースポイントを追加することができます。

