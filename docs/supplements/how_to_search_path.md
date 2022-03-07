# パス探索の高速化

`search_paths`メソッドはノード図を描き、始点に設定したノードから終点に設定したノードまでを一筆書きで辿れる経路（パス）を全探索し、
見つかった経路（パス）を全て返却する関数です。

ノード図から全ての探索できるパスを全探索するため、対象のアプリケーションによっては処理に時間がかかりすぎてしまうケースがあります。  
このようなケースで、高速に目的とするパスを探索する方法を説明します。


## API

search_pathsでは、このような時間がかかりすぎてしまうケースを避けるためのAPIが備わっています。  

```
search_paths(
    start_node_name: 'str',
    end_node_name: 'str',
    max_node_depth: 'Optional[int]' = None,
    node_filter: 'Optional[Callable[[str], bool]]' = None,
    communication_filter: 'Optional[Callable[[str], bool]]' = None
) -> 'List[PathStructValue]'
```

- start_node_name : 探索の始点となるノード名
- end_node_name : 探索の終点となるノード名
- max_node_depth : 探索するパスが含む最大ノード数。小さいほど探索範囲が狭くなります
- node_filter: 探索対象のノードをフィルタする関数。Trueを返却した場合は探索対象になります。
- communication_filter: 探索対象の通信をフィルタする関数。Trueを返却した場合は探索対象になります。

## 例

```
import re

# 正規表現に一致するノード名を探索から除外する
node_filters = [
    re.compile(r'/_ros2cli_/*'),
    re.compile(r'/launch_ros_*'),
]

# 正規表現に一致するトピック名を探索から除外する
comm_filters = [
    re.compile(r'/tf/*'),
]
def comm_filter(topic_name: str) -> bool:
    can_pass = True
    for comm_filter in comm_filters:
        can_pass &= not bool(comm_filter.search(topic_name))
    return can_pass

def node_filter(node_name: str) -> bool:
    can_pass = True
    for node_filter in node_filters:
        can_pass &= not bool(node_filter.search(node_name))
    return can_pass

paths = arch.search_paths(
    '/start_node',
    '/end_node',
    max_node_depth=30,
    node_filter = node_filter,
    communication_filter = comm_filter)
```