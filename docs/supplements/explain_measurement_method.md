# End-to-Endレイテンシの測定方法
CARETでは通信レイテンシ（Pub/Subレイテンシ）とノードレイテンシを結びつけてE2Eレイテンシを算出しています。
ここでは通信レイテンシとノードレイテンシの算出方法について説明します。


## 通信レイテンシとノードレイテンシの算出方法
以下は通信レイテンシとノードレイテンシを表した図です。

![callback_and_node_latency](/imgs/callback_and_node_latency.png)


### 通信レイテンシの算出方法
通信レイテンシはコールバックのpublishの時刻と、後続のコールバックが開始した時刻の差分で計算しています。

#### プロセス内通信
次の表はプロセス内通信のトレースに必要なトレースポイントと、そのトレースポイントの引数（トレースデータとして出力されるもの）です。※簡易版


| トレースポイント名                | 引数1     |  引数2        | 時刻          |
|--------------------------|----------------------|------------------|-----------|
| rclcpp_intra_publish | publisher_handle_arg | <span style="color: red; ">message_arg</span>      | time1          |
| dispatch_intra_process_subscription_callback | <span style="color: red; ">message_arg</span>             |  <span style="color: green; ">callback_arg</span>     | time2 |
| callback_start                               | <span style="color: green; ">callback_arg</span>         | is_intra_process |   time3        |

※[引数についてはこちらを参照](https://tier4.github.io/CARET_doc/design/tracepoint_definition/)

message_argとcallback_argにはメッセージのアドレス、コールバックのアドレスが格納されています。
これらを使うことによりpublishからcallback_startまでのトレースデータの紐づけを行い、下記のような表を作成します。
最後に`callback_start - rclcpp_intra_publish`でプロセス内通信のレイテンシを算出しています。

|idx| rclcpp_intra_publish | dispatch_intra_process_subscription_callback | callback_start |
|-|-|-|-|
|0| time1 | time2 | time3 |
|1| ... | ... | ... |


#### プロセス間通信
以下はプロセス間通信のトレースポイントとその引数です。

| トレースポイント名 | 引数1  | 引数2 | 引数3 | 時刻 |
|-|-|-|-|-|
| rclcpp_publish  | publisher_handle_arg | <span style="color: red; ">message_arg</span>@1   |   | time1 |
| rcl_publish  | publisher_handle_arg | <span style="color: red; ">message_arg</span>@1   |   | time2 |
| dds_write   | publisher_handle_arg | <span style="color: red; ">message_arg</span>@1   |    | time3 |
| dds_bind_addr_to_stamp    | <span style="color: red; ">message_arg</span>@1       | <span style="color: green; ">stamp_arg</span>        |  | time4 |
| dispatch_subscription_callback | messsage_arg@2       | <span style="color: green; ">stamp_arg</span>        | <span style="color: blue; ">callback_arg</span>          |  time5 |
| callback_start   | <span style="color: blue; ">callback_arg</span>   | is_intra_process |  |  time6 |


プロセス内通信と同様に、publishからcallback_startまでの表を作成します（下記表）。
1行が1つのプロセス間通信のチェーンを表し、`callback_start - rclcpp_publish` にてプロセス間通信のレイテンシを算出します。

| idx | rclcpp_publish | rcl_publish | dds_write | dds_bind_addr_to_stamp | dispatch_subscription_callback | callback_start |
|-|-|-|-|-|-|-|
| 0 | time1 | time2 | time3 | time4 | time5 | time6 |
| 1 | ... | ... | ... | ... | ... | ...　|


## ノードレイテンシの算出方法
ノードレイテンシの算出方法は2つあります。
それぞれの手法・長所・短所を説明します。

### コールバックチェーンの利用
コールバックレイテンシの測定は、同じコールバックアドレスを持つcallback_startとrclcpp_publishの差分を使って算出します。
callback_startとrclcpp_publishの紐づけは、rclcpp_publishから見て一番近いcallback_startと紐づけます。※そのためSingle Threaded Executorでしか計算できません。

| idx | callback_start (CB_A) [s] | rclcpp_publish (CB_A) [s] | callback_arg (CB_A) |
|-|-|-|-|
| 0 | 0 | 3 | 0x1000 |
| 1 | 2 | 5 | 0x1000 |
| 2 | 4 | 7 | 0x1000 |
| ... | ... | ... | ... |

| idx | callback_start (CB_B) [s] | rclcpp_publish (CB_B) [s] | callback_arg (CB_B) |
|-|-|-|-|
| 0 | 4 | 8 | 0x2000 |
| 1 | 8 | 12 | 0x2000 |
| ... | ... | ... | ... |

上記表のようにコールバックA・Bが存在し、A→Bと処理が続く時、コールバックチェーンは下記表のように一つのテーブルにできます。
| idx | callback_start (CB_A) [s] | rclcpp_publish (CB_A) [s] | callback_start (CB_B) [s] | rclcpp_publish (CB_B) [s] |
|-|-|-|-|-|
| 0 | 0 | 3 | 4 | 8 |
| 1 | 2 | 5 | Lost | Lost |
| 2 | 4 | 7 | 8 | 12 |
| ... | ... | ... | ... | ... |

上記のようにコールバックチェーンをつなぎ、ノードレイテンシを算出する。



**想定**
  - コールバック間のキューサイズ1を想定
  - 厳密に測定できるのはSingle Threaded Executorのみ

**長所**
 - ソースコードへの変更が不要
 - 任意のメッセージ型に適用可能

**短所**

 - キューサイズが大きい時⇒レイテンシが小さめに出るケースがある。
 - multi threaded executor では、レイテンシが大きめに出るケースがある。
 - 実装を読み解くのが難しい


### 入出力のヘッダータイムスタンプのマッチング
パブリッシュされるトピックからヘッダースタンプを取得し、その差分からノードレイテンシを算出します。


**想定**

 - 入出力のタイムスタンプの値でマッチングが取れること（値を書き換えずに、そのまま publish していること）
 - 入出力ともに header をもっていること

**長所**

 - ソースコードへの変更が不要
 - キューサイズに依らない

**短所**
 - ヘッダーが必要
 - publish 前に stamp=現在時刻としているケースは対応不可







