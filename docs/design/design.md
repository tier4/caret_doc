# アーキテクチャ

CARETのアーキテクチャを以下に示します。

![architecture](../../imgs/architecture.png)

CARETはアプリケーション（App）、ROS、DDSのレイヤーにトレースポイントを埋め込むことで振舞いをTrace Dataとして記録します。  
Trace Dataに記録された情報は、CARET_analyzeにより、実行毎に不変な静的な情報（例：ノードの情報）を有するArchitectureと、実行毎に可変な動的な情報（例：実行時間）を有するRuntime Dataに格納されます。  
Architecture と Runtime Dataは、ノードやエグゼキュータなどのROSの概念を意識したクラス構造を有する python のクラス群です。  
ROSの概念を意識したクラス構造にすることで、通信／ノード／エグゼキュータといった評価対象を探しやすくしています。  
Architectureはパスの探索やノードレイテンシの定義を行うAPIを提供しており、yamlファイルへの保存および読み込みが可能で、評価毎に再利用が可能です。  
Runtime Dataは、コールバックの実行時間や通信レイテンシなどをpandas DataFrame型で取得できるAPIを提供しており、目的に合わせた評価や解析が行えます。  
また、CARET_analyzeは、可視化モジュール（Plot）も提供しており、少ない手間で可視化が可能になっています。

