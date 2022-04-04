# 測定

サンプルのアプリケーションを例に、ツールの使い方を説明していきます。

サンプルのリポジトリ<https://github.com/tier4/CARET_demos.git>

## アプリケーションのビルド

caret の環境変数を`/opt/ros/galactic/setup.bash`の後に設定し、測定対象のアプリケーションをビルドします。  
caret 導入前にビルド済みのアプリケーションも、再度ビルドが必要になります。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
source ~/ros2_caret_ws/install/local_setup.bash

colcon build --symlink-install --packages-up-to caret_demos
```

> 測定対象のアプリケーションも再ビルドが必要な理由
>
> caret は rclcpp のヘッダーで読み込まれるテンプレート実装内にもトレースポイントを追加しています。  
> トレースポイントを追加された rclcpp を有効にするため、測定対象のアプリケーションを再度ビルドする必要が有ります。  
> 従って、 apt でインストールした demo_nodes_cpp などのアプリケーションは測定することが出来ません。  
> 測定するにはソースファイルからビルドし直す必要があります。

## 測定

### LTTngセッションの開始

CARETは測定のためにLTTngを利用しており、LTTngセッションの開始が必要です。  
ここでは、ros2 tracingによって用意されたインターフェース経由でLTTngセッションを開始します。


```bash
source /opt/ros/galactic/setup.bash

# トレースファイルの出力先を設定。デフォルトは ~/.ros/tracing。
mkdir -p ~/ros2_ws/evaluate
export ROS_TRACE_DIR=~/ros2_ws/evaluate
ros2 trace -s e2e_sample -k -u "ros2*"
# Enter を押して測定開始

# -s はセッション名。
# 上記の例では~/ros_ws/evaluate/e2e_sampleに測定結果が保存されます。
```

別途ターミナルでLTTngセッションを開始せずに、Launchファイル内でLTTngのセッションを開始する方法もあります。  
[LTTng セッションの開始方法](../supplements/how_to_run_lttng_session.md)を参照してください。

### アプリケーションの開始・測定

アプリケーションを実行するターミナルでは、caret_ws 配下の rclcpp を使わせるため、opt 配下の後に caret_ws 配下の setup.bash を実行します。

```bash
source /opt/ros/galactic/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash
```

以下のコマンドでビルドした rclcpp が使われることを確認します。

```
ldd ./build/caret_demos/end_to_end_sample  | grep rclcpp

# librclcpp.so => /home/user_name/ros2_caret_ws/install/rclcpp/lib/librclcpp.so
```

フックを有効にするため、`LD_PRELOAD`を設定します。

```bash
# フックで追加されるトレースポイントの有効化
export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
```

不要なノードやトピックを測定対象から除外する設定を行います。  
トレースフィルタリング機能については、[トレースフィルタリング](../supplements/trace_filtering.md)を参照。


```bash
export CARET_IGNORE_NODES="/rviz*"
export CARET_IGNORE_TOPICS="/clock:/parameter_events"
```
```bash
ros2 launch caret_demos end_to_end_sample.launch.py
^C # 数秒後、 Ctrl+C で終了

[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[end_to_end_sample-1] [INFO] [1631481246.160958444] [rclcpp]: signal_handler(signal_value=2)
[INFO] [end_to_end_sample-1]: process has finished cleanly [pid 722356]
```

終了する際の順番に指定はありません。  
`ros2 trace` を実行しているターミナルは `Enter` を再度押して測定を終了してください。

## 測定結果の確認

記録されたトレースポイントを確認します。

```bash
# 以下のトレースポイントが記録されていることを確認
$ babeltrace ~/ros2_ws/evaluate/e2e_sample/ | cut -d' ' -f 4 | sort -u
ros2:callback_end:
ros2:callback_start:
ros2_caret:add_callback_group_static_executor:
ros2_caret:callback_group_add_service:
ros2_caret:callback_group_add_subscription:
ros2_caret:callback_group_add_timer:
ros2_caret:construct_static_executor:
ros2_caret:dds_bind_addr_to_stamp:
ros2_caret:dds_write:
ros2_caret:rmw_implementation:
ros2:dispatch_subscription_callback:
ros2:rclcpp_callback_register:
ros2:rclcpp_publish:
ros2:rclcpp_service_callback_added:
ros2:rclcpp_subscription_callback_added:
ros2:rclcpp_subscription_init:
ros2:rclcpp_timer_callback_added:
ros2:rclcpp_timer_link_node:
ros2:rcl_init:
ros2:rcl_node_init:
ros2:rcl_publish:
ros2:rcl_publisher_init:
ros2:rcl_service_init:
ros2:rcl_subscription_init:
ros2:rcl_timer_init:
```

トレースポイントが不足している場合、セットアップで問題がある可能性があります。  
トレース結果の保存が間に合わない場合には、`Tracer discarded とエラーが出る場合`とエラーが出る場合があります。

### Tracer discarded とエラーが出る場合
babeltrace コマンドの実行時に以下のような警告が現れる場合、  
トレース結果のドロップが生じており、解析が正しく行えません。  

トレースフィルタリング機能の設定を見直し、警告が出ないようにしてください。

```bash
[warning] Tracer discarded 328 events between [10:46:15.566916889] and [10:46:15.620323777]  
in trace UUID 353a72bc12d4bcc85c9158dd8f88ef9, at path: "end_to_end_sample/ust/uid/10368/64-bit",  
within stream id 0, at relative path: "ros2_3".  
You should consider recording a new trace with larger buffers or with fewer events enabled.
```


