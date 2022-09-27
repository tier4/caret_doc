# Recording with CARET

## Set LD_PRELOAD

```sh
# Enable tracepoints which are defined hooked functions.
export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
```

## Start LTTng session

### 別途ターミナルで開始

ターミナル上でセッションを開始できます。  
ros2 trace や caret は ROS アプリケーションの初期化時に呼ばれる関数にもトレースポイントを挿入しています。

以下の順番で起動を行ってください。

1. セッション開始

   ```bash
   ros2 trace -s end_to_end_sample -k -u "ros2*"
   # ヘルプは ros2 trace -h
   ```

2. アプリケーションの実行

   ```bash
   $ ros2 launch caret_demos end_to_end_sample.launch.py
   　# end_to_end_sample.launch.py には tracetools_launch.action.Trace が追加済みです。
     # 自動でros2 trace同等の事を行ってくれるので、ros2 trace コマンドの実行は不要です。
   ```

終了する際の順番に指定はありません。  
`ros2 trace` を実行しているターミナルは `Enter` を再度押して終了してください。

### Launch システムでの開始

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
