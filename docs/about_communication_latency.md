# 通信レイテンシについて
通信レイテンシとして、以下の２種類が測定できます
- DDS-layer レイテンシ
- Pub-Sub レイテンシ
また、プロセス内通信とプロセス間通信により、意味が異なってきます。
それぞれの違いについて説明します。

※ DDS-layer レイテンシは現在試験導入段階です。正しい値が測定されないケースがありますのでご了承ください。

## レイテンシの定義
![communication_latency](/imgs/communication_latency.png)

### Pub-Sub レイテンシの定義
Pub-Subレイテンシの定義は以下の通りです。

```
Pub-Sub レイテンシ = Subscriptionコールバックの実行開始時刻 - Publisher::publish()実行開始時刻
```
この値は、ros2 topic delay や、topic statisticsで測定される値に対応します。

```
通信レイテンシ = 現在時刻 - msg.header.stamp
```

また、プロセス内通信とプロセス間通信で、レイテンシの構成が変わってきます。
Pub-Subレイテンシはプロセス内通信・プロセス間通信両方で算出されます。

Pub-Subレイテンシは、ユーザーコード（コールバック）の実行以外によるレイテンシを全て含んだ値です。
概ねコールバック終了からコールバック開始までの時間です。
publish後、subscriptionコールバック実行開始までは、エグゼキューターによるスケジューリングの遅延が発生します。
プロセス間通信に対するPub-Subレイテンシとは、このスケジューリングによる遅延のことをさします。
大まかなシーケンス図を書きます。

DDSによるレイテンシと、エグゼキューターによる遅延を指します


### DDS-layer レイテンシの定義

DDS-layer レイテンシの定義は以下の通りです。

```
DDS-layer レイテンシ = on_data_availableの実行開始時刻 - dds_writeの実行開始時刻
```

DDS-layerレイテンシは、プロセス間通信のみ算出されます。

dds_writeはrmwがDDSに書き込む時刻、
on_data_availableはDDSがRMWに通知する時刻になります。

※ CycloneDDS はon_data_availableをエグゼキューターの起床に使用していません。
そのため、on_data_availaleとsubscription callbackの実行開始の前後の時間関係がズレる可能性があります。

## Pub-Sub レイテンシと DDS-layer レイテンシの違い
Pub-SubレイテンシとDDS-layerレイテンシはどちらも通信レイテンシと捉えることができます。

レイテンシの観測開始地点である、rclcpp_publishとdds_writeは同じスレッドで実行されており、ほとんど差はありません。
 callback_startはコールバックスレッドが実行します。そのため、他のコールバックが実行完了し、対象のSubscriptionコールバックの実行開始順番が回ってくるまで待たされる可能性があります。
例えば、タイマーコールバックとsubscriptionコールバックがほぼ同時に発火した場合、Pub-subレイテンシにはタイマーコールバックの実行時間も含まれます。　

一方で、観測の終了地点である、on_data_availableとcallback_startは、実行するスレッドが異なります。
on_data_availableはDDSスレッドが実行します。 従って、DDS-layerレイテンシは他のコールバックの実行時間は含みません。

Pub-Subレイテンシに大きな値が算出された場合、DDS-Layerレイテンシも確認することで、他のコールバックによる影響か判断することができます。


エグゼキュータのスケジューリングについては、こちらの論文も参照ください。
[Response-Time Analysis of ROS 2 Processing
Chains Under Reservation-Based Scheduling](https://drops.dagstuhl.de/opus/volltexte/2019/10743/pdf/LIPIcs-ECRTS-2019-6.pdf)


