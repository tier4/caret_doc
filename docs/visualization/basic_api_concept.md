# Basic APIs Concept

## CARET の可視化方針

CARET はたくさんの可視化 API を提供している。
図 1 は API の設計方針を示している。

（図 1）横軸が測定粒度で縦軸がシステムレベルの図

横軸は可視化粒度を、縦軸は計測対象システムの粒度を表す。  
左側にある図ほど、より細かい情報を可視化する図であり、下に行くほど可視化対象のシステムが細かい。

CARET の特徴の一つとして、可視化・対象システムの粒度を柔軟に変更しながらシステムの評価を行える。
これにより、可視化結果でシステムに問題が見られた場合は、図の左下に向かって可視化を繰り返す事で、問題箇所の特定を支援する。

## Plot クラス（可視化クラス）の設計方針

CARET が提供する可視化ツールは Plot クラスで実装されている（一部違うものもある）。
Plot クラスの主な基本設計を紹介する。

Plot クラスの各 API の使い方は以下で統一されている。

```python3
from caret_analyze.plot import Plot

... # Processing input data

plot = Plot.create_[Where_Visualize]_[How_Visualize]_plot(data)
plot.show()
plot.to_dataframe()
```

返り値である plot は show()と to_dataframe()の 2 つの関数を持つ。
show()関数は図を出力する。
時系列的にシステムの挙動を把握できる。
to_dataframe()はデータをまとめた表を返す。
具体的な数値をもとに解析を行いたいときに活用することを想定している。

Note: 詳細な入出力・オプションに関しては[こちら](https://tier4.github.io/CARET_analyze/latest/plot/#caret_analyze.plot.TimeSeriesPlot)を参照。
