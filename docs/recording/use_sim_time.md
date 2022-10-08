# sim_time での時系列データ評価

CARET のデフォルト（LTTng）では、システム時間で測定結果が出力されます。  
この場合、同じ条件で繰り返し測定する場合などでは、時間がズレてしまい、並べて比較するのが困難です。  
そのような場合には、sim_time で時刻を一致させることで、同条件下での評価が可能になります。  
本ページでは、時系列データの時刻を sim_time に時刻を揃えて評価する方法を説明します。

## sim_time の記録

sim_time を使った評価を行う際には、測定時に特定のノードを実行させておく必要が有ります。

```bash
ros2 run caret_trace clock_recorder
```

`clock_recorder`ノードを起動させておくことで、`ros2_caret:simime`トレースが有効になります。

以下のようにコマンドを実行することで、トレースポイントが記録されていることを確認できます。

```bash
$ babeltrace ~/.ros/tracing/ctf_path | cut -d' ' -f 4 | sort -u | grep sim_time
ros2_caret:sim_time:
```

なお、sim_time は`/clock`トピックが publish されている間のみ記録されます。

## sim_time を使った可視化

`caret_analyze.plot`配下の可視化では、時間軸を sim_time にした可視化が可能です。  
最新版 v.0.2.1 では、`callback_sched`と`message_flow`が sim_time の時刻を使用した可視化に対応しています。

```python
def callback_sched(
    target: Union[Node, CallbackGroup, Executor],
    lstrip_s: float = 0,
    rstrip_s: float = 0,
    coloring_rule='callback',
    use_sim_time: bool = False)
):

def message_flow(
    path: Path,
    export_path: Optional[str] = None,
    granularity: Optional[str] = None,
    lstrip_s: float = 0,
    rstrip_s: float = 0,
    use_sim_time: bool = False
)
```

オプション引数の`use_sim_time = True`とすることで、横軸が sim_time のグラフを描画できます。  
なお、`ros2_caret:sim_time`トレースポイントが見つからない場合には、エラーが出るようになっています。
