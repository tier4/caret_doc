# 測定

サンプルを例に、ツールの使い方を説明していきます。

## サンプルのビルド

caret の環境変数を`/opt/ros/galactic/setup.bash`の後に設定し、測定対象のアプリケーションをビルドします。caret 導入前にビルド済みのアプリケーションも、再度ビルドが必要になります。

```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
$ git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
$ source ~/ros2_caret_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-up-to caret_demos
```

>  ビルドが必要な理由：
>
> caret は rclcpp のヘッダーで読み込まれるテンプレート実装内にもトレースポイントを追加しています。
> トレースポイントを追加された rclcpp を有効にするため、測定対象のアプリケーションを再度ビルドする必要が有ります。
> 従って、 apt でインストールした demo_nodes_cpp などのアプリケーションは測定することが出来ません。測定するにはソースファイルからビルドし直す必要があります。

## 測定の設定

###  Launch ファイルの編集

LTTng のセッション開始を launch ファイルに記述します。
launch ファイル内へ、`tracetools_launch.action.Trace` を追加します。

CARET_demos の launch ファイルでは記述済みです。
参考のために追加箇所を示します。

```python
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

#### Launch ファイル以外の方法で LTTng セッションを開始する方法

ターミナル上でセッションを開始できます。
ros2 trace や caret は ROS アプリケーションの初期化時に呼ばれる関数にもトレースポイントを挿入しているので、以下の順番で起動を行ってください。

1. セッション開始

   ```bash
   $ ros2 trace -s end_to_end_sample -k -u "ros2*"
   $ # ヘルプは ros2 trace -h
   ```

2. アプリケーションの実行

   ```bash
   $ ros2 launch caret_demos end_to_end_sample.launch.py
   　# end_to_end_sample.launch.py には tracetools_launch.action.Trace が追加済みなので、
     # ros2 trace コマンドは不要
   ```

## 測定

launch ファイルを実行します

```bash
$ source /opt/ros/galactic/setup.bash
$ # caret_ws 配下の rclcpp を使わせるため、opt 配下の後に caret_ws 配下の setup.bash を実行する。
$ source ~/ros2_caret_ws/install/local_setup.bash
$ source ~/ros2_ws/install/local_setup.bash
$ export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
$ mkdir -p ~/ros2_ws/evaluate
$ export ROS_TRACE_DIR=~/ros2_ws/evaluate # トレースファイルの出力先を設定。デフォルトは ~/.ros/tracing。
$ # ビルドした rclcpp が使われることを確認。
$ ldd ./build/caret_demos/end_to_end_sample  | grep rclcpp
  librclcpp.so => /home/user_name/ros2_caret_ws/install/rclcpp/lib/librclcpp.so
$ ros2 launch caret_demos end_to_end_sample.launch.py
^C # 数秒後、 Ctrl+C で終了
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[end_to_end_sample-1] [INFO] [1631481246.160958444] [rclcpp]: signal_handler(signal_value=2)
[INFO] [end_to_end_sample-1]: process has finished cleanly [pid 722356]
```

設定可能な環境変数については、 [利用可能な環境変数一覧](./env.md) を参照。

記録されたトレースポイントを確認します。

```bash
$ # 以下のトレースポイントが記録されていることを確認
$ babeltrace ~/ros2_ws/evaluate/end_to_end_sample/ | cut -d' ' -f 4 | sort -u
ros2:callback_end:
ros2:callback_start:
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


# babeltrace コマンドの実行時に以下のような警告が現れる場合、
# トレース結果のドロップが生じており、解析が正しく行えません。
# caret が備えているトレースログのフィルタリング機能を使い、警告が出ないようにしてください。
# フィルタリング機能については、「環境変数について」を参照。
[warning] Tracer discarded 328 events between [10:46:15.566916889] and [10:46:15.620323777] in trace UUID 353a72bc12d4bcc85c9158dd8f88ef9, at path: "end_to_end_sample/ust/uid/10368/64-bit", within stream id 0, at relative path: "ros2_3". You should consider recording a new trace with larger buffers or with fewer events enabled.
```
