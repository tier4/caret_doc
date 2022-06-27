# レイテンシの計算方法

## 通信レイテンシとノードレイテンシの算出方法

以下は通信レイテンシとノードレイテンシを表した図です。

![callback_and_node_latency](../imgs/callback_and_node_latency.png)

ここでは通信レイテンシとノードレイテンシの算出方法について説明します。

### 通信レイテンシの算出方法

通信レイテンシはコールバックの publish の時刻と、後続のコールバックが開始した時刻の差分で計算しています。

#### プロセス内通信

次の表はプロセス内通信のトレースに必要なトレースポイントと、そのトレースポイントの引数（トレースデータとして出力されるもの）です。※簡易版

| トレースポイント名                           | 引数 1                                           | 引数 2                                           | 時刻  |
| -------------------------------------------- | ------------------------------------------------ | ------------------------------------------------ | ----- |
| rclcpp_intra_publish                         | publisher_handle_arg                             | <span style="color: red; ">message_arg</span>    | time1 |
| dispatch_intra_process_subscription_callback | <span style="color: red; ">message_arg</span>    | <span style="color: green; ">callback_arg</span> | time2 |
| callback_start                               | <span style="color: green; ">callback_arg</span> | is_intra_process                                 | time3 |

※[引数についてはこちらを参照](../design/supported_tracepoints.md)

message_arg と callback_arg にはメッセージのアドレス、コールバックのアドレスが格納されています。
rclcpp_intra_publisher と dispatch_intra_process_subscription_callback は同じ message_arg を持つトレースポイント同士で紐づけ、dispatch_intra_process_subscription_callback と callback_start は同じ callback_arg を持つトレースポイント同士で紐づけることにより、下記のような表を作成します。
最後に`callback_start - rclcpp_intra_publish`で**プロセス内通信のレイテンシ**を算出しています。

| idx | rclcpp_intra_publish | dispatch_intra_process_subscription_callback | callback_start |
| --- | -------------------- | -------------------------------------------- | -------------- |
| 0   | time1                | time2                                        | time3          |
| 1   | ...                  | ...                                          | ...            |

#### プロセス間通信

以下はプロセス間通信のトレースポイントとその引数です。

| トレースポイント名             | 引数 1                                          | 引数 2                                          | 引数 3                                          | 時刻  |
| ------------------------------ | ----------------------------------------------- | ----------------------------------------------- | ----------------------------------------------- | ----- |
| rclcpp_publish                 | publisher_handle_arg                            | <span style="color: red; ">message_arg</span>@1 |                                                 | time1 |
| rcl_publish                    | publisher_handle_arg                            | <span style="color: red; ">message_arg</span>@1 |                                                 | time2 |
| dds_write                      | publisher_handle_arg                            | <span style="color: red; ">message_arg</span>@1 |                                                 | time3 |
| dds_bind_addr_to_stamp         | <span style="color: red; ">message_arg</span>@1 | <span style="color: green; ">stamp_arg</span>   |                                                 | time4 |
| dispatch_subscription_callback | messsage_arg@2                                  | <span style="color: green; ">stamp_arg</span>   | <span style="color: blue; ">callback_arg</span> | time5 |
| callback_start                 | <span style="color: blue; ">callback_arg</span> | is_intra_process                                |                                                 | time6 |

プロセス内通信と同様に同じ引数を持つトレースポイント同士を紐づけていき、publish から callback_start までの表を作成します（下記表）。  
1 行が 1 つのプロセス間通信のチェーンを表し、`callback_start - rclcpp_publish` にて**プロセス間通信のレイテンシ**を算出します。

| idx | rclcpp_publish | rcl_publish | dds_write | dds_bind_addr_to_stamp | dispatch_subscription_callback | callback_start |
| --- | -------------- | ----------- | --------- | ---------------------- | ------------------------------ | -------------- |
| 0   | time1          | time2       | time3     | time4                  | time5                          | time6          |
| 1   | ...            | ...         | ...       | ...                    | ...                            | ...　          |

## ノードレイテンシの算出方法

ノードレイテンシの算出方法は 2 つあります。
それぞれの手法・長所・短所を説明します。

### コールバックチェーンの利用

コールバックレイテンシの測定は、同じコールバックアドレスを持つ callback_start と rclcpp_publish の差分を使って算出します。  
callback_start と rclcpp_publish の紐づけは、rclcpp_publish から見て一番近い callback_start と紐づけます。

| idx | callback_start (cb_A) [s] | rclcpp_publish (cb_A) [s] | callback_end (cb_A) [s] | callback_arg (cb_A) |
| --- | ------------------------- | ------------------------- | ----------------------- | ------------------- |
| 0   | 0                         | 3                         | 4                       | 0x1000              |
| 1   | 2                         | 5                         | 6                       | 0x1000              |
| 2   | 4                         | 7                         | 8                       | 0x1000              |
| ... | ...                       | ...                       | ...                     | ...                 |

| idx | callback_start (cb_B) [s] | rclcpp_publish (cb_B) [s] | callback_end (cb_B) [s] | callback_arg (cb_B) |
| --- | ------------------------- | ------------------------- | ----------------------- | ------------------- |
| 0   | 4                         | 8                         | 9                       | 0x2000              |
| 1   | 8                         | 12                        | 13                      | 0x2000              |
| 2   | 10                        | 14                        | 15                      | 0x2000              |

上記表のようにコールバック A・B（cb_A・cb_B）が存在し、A→B と処理が続く時、cb_A の callback_end と cb_B の callback_start を結び付けて表を作ります。  
最後の callback だけは publish の時の時刻を採用し、下記表のように一つのテーブルにします。

| idx | callback_start (cb_A) [s] | callback_end (cb_A) [s] | callback_start (cb_B) [s] | rclcpp_publish (cb_B) [s] |
| --- | ------------------------- | ----------------------- | ------------------------- | ------------------------- |
| 0   | 0                         | 4                       | 4                         | 8                         |
| 1   | 2                         | 6                       | Lost                      | Lost                      |
| 2   | 4                         | 8                       | 8                         | 12                        |
| ... | ...                       | ...                     | ...                       | ...                       |

上記のようにコールバックチェーンをつなぎ、`rclcpp_publish (cb_X) - callback_start (cb_Y)`で**ノードレイテンシ**を算出します。

> ※ノード内にコールバックが１つの場合、X, Y は同じものを指します。  
> 複数コールバックがある場合は、X が最後・Y が最初のコールバックを指します。
