# 測定

サンプルを例に、ツールの使い方を説明していきます。

## サンプルのビルド

caret の環境変数を設定し、測定対象のアプリケーションをビルドします。

```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
$ git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
$ source ~/ros2_caret_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-up-to caret_demos
```

>  ビルドが必用な理由：
>
> caret は rclcpp のテンプレート実装内（ヘッダーで読み込まれる）にトレースポイントを追加しています。
> トレースポイントを追加された rclcpp を有効にするため、測定対象のアプリケーションをビルドする必用が有ります。
> apt でインストールした demo_nodes_cpp などのアプリケーションは測定することが出来ません。測定するにはソースファイルからビルドし直す必用があります。

## 測定の設定

### 環境変数の設定

測定に必用な環境変数を設定します。
設定可能な環境変数は「利用可能な環境変数一覧」を参照。

```bash
$ source ~/ros2_ws/install/local_setup.bash
$ export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
$ mkdir ~/ros2_ws/evaluate
$ export ROS_TRACE_DIR=~/ros2_ws/evaluate # トレースファイルの出力先を設定。デフォルトは ~/.ros/tracing。
```

###  Launch ファイルの編集

LTTng のセッション開始を launch ファイルに記述します。
launch ファイル内のへ、tracetools_launch.action.Trace を追加します。

CARET_demos の launch ファイルでは記述済みです。
参考のために追加箇所を示します。

```bash
$ cat ~/ros2_ws/src/CARET_demos/caret_demos/launch/end_to_end_sample.launch.py
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace

def generate_launch_description():
    return launch.LaunchDescription([
    	## Trace 追加 -- ここから --
        Trace(
            session_name='end_to_end_sample',　# LTTng のセッション名。
            events_kernel=[], # カーネルイベントは無効化
            events_ust=['ros2*'] # ros2 と caret のトレースポイントを有効化]
        ),
        ## Trace 追加 -- ここまで --
        launch_ros.actions.Node(
            package='caret_demos', executable='end_to_end_sample', output='screen'),
    ])
```



#### Launch ファイルの編集が難しい場合

launch ファイルへの記述が難しい場合、ターミナル上でセッションを開始できます。
ros2 trace や caret は初期化関数に挿入したトレースポイントも利用しているので、以下の順番で起動を行ってください。

1. セッション開始

   ```bash
   $ ros2 trace -s end_to_end_sample -k -u ros2*
   ```

2. アプリケーションの実行

   ```bash
   $ ros2 launch caret_demos end_to_end_sample.launch.py
   ```

## 測定

launch ファイルを実行します

```bash
$ ros2 launch caret_demos end_to_end_sample.launch.py
^C # 数秒後、 Ctrl+C で終了
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[end_to_end_sample-1] [INFO] [1631481246.160958444] [rclcpp]: signal_handler(signal_value=2)
[INFO] [end_to_end_sample-1]: process has finished cleanly [pid 722356]
```

記録されたトレースポイントの確認します。

```bash
$ # 以下のトレースポイントが記録されていることを確認
$ babeltrace ~/ros2_ws/evaluate/end_to_end_sample/ | cut -d' ' -f 4 | sort -u
ros2:callback_end:
ros2:callback_start:
ros2_caret:dds_bind_addr_to_stamp:
ros2_caret:dds_write:
ros2_caret:on_data_available:
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

# babeltrace コマンドの実行時に以下のような警告が現れる場合、トレース結果のドロップが生じており、測定結果が正しく表示さない。
# caret が備えているトレースログのフィルタリング機能を使い、警告が出ないようにすること。
# フィルタリング機能については、「環境変数について」を参照のこと。
[warning] Tracer discarded 328 events between [10:46:15.566916889] and [10:46:15.620323777] in trace UUID 353a72bc12d4bcc85c9158dd8f88ef9, at path: "end_to_end_sample/ust/uid/10368/64-bit", within stream id 0, at relative path: "ros2_3". You should consider recording a new trace with larger buffers or with fewer events enabled.
```
