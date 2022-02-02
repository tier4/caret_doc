# LTTngEventFilterの使用

CARETは数分程度の測定や解析までを取り扱うことができます。  
より長時間、十数分〜数十分程度の測定結果を取り扱う際には、メモリの使用量・解析時間・可視化時間などが問題となってきます。

そこで、CARETはLTTngの読み込み時に不要な測定結果を省くためのAPI(LTTngEventFilter)を備えています。  
ここでは、LTTngEventFitlerの使い方について説明します。

LTTngEventFitlerでは、以下のフィルタを備えています。

- init_pass_filter
- duration_filter
- strip_filter

## API説明


```
LttngEventFilter.init_pass_filter()
```

- 初期化時のトレースポイントのみを通します。


```
LttngEventFilter.duration_filter(duration_s: float, offset_s: float)
```

- [duration_s]秒間を測定対象にフィルタリング
- 最初の[offset_s]秒間を無視

```
LttngEventFilter.strip_filter(lsplit_s: Optional[float], rsplit_s: Optional[float])
```

- 測定開始から[lsplit_s]秒間の測定結果を無視
- 測定測定終了から[rsplit_s]秒間の測定結果を無視

## 使用例

```
from caret_analyze import Lttng, LttngEventFilter

lttng = Lttng('/path/to/ctf', event_filters=[
  LttngEventFilter.duration_filter(10, 5)
]) # 測定開始5秒後から10秒間にフィルタいrング
```
event_filtersは複数指定することもできます。  
複数を入れた場合は、全てのフィルタが指定した区間のみにフィルタリングされます。
