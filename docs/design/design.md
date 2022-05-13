# アーキテクチャ

CARET のアーキテクチャを以下に示します。

![architecture](../../imgs/architecture.png)

CARET はアプリケーション（App）、ROS、DDS のレイヤーにトレースポイントを埋め込むことで振舞いを Trace Data として記録します。  
Trace Data に記録された情報は、CARET_analyze により、実行毎に不変な静的な情報（例：ノードの情報）を有する Architecture と、実行毎に可変な動的な情報（例：実行時間）を有する Runtime Data に格納されます。  
Architecture と Runtime Data は、ノードやエグゼキュータなどの ROS の概念を意識したクラス構造を有する python のクラス群です。  
ROS の概念を意識したクラス構造にすることで、通信／ノード／エグゼキュータといった評価対象を探しやすくしています。  
Architecture はパスの探索やノードレイテンシの定義を行う API を提供しており、yaml ファイルへの保存および読み込みが可能で、評価毎に再利用が可能です。  
Runtime Data は、コールバックの実行時間や通信レイテンシなどを pandas DataFrame 型で取得できる API を提供しており、目的に合わせた評価や解析が行えます。  
また、CARET_analyze は、可視化モジュール（Plot）も提供しており、少ない手間で可視化が可能になっています。
