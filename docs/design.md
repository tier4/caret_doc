#

DDS レイテンシを算出するにあたって





CycloneDDSはon_data_availableを実行していない。

そのため、フラグを操作し、強制的にon_data_availableを実行させるようにした。



ただし、on_data_availableはrmw_waitの起床とは別物なので、

on_data_available -> rmw_wait  起床とはならないケースがある。

従って、cycloneDDS のDDSレイテンシは参考程度に考えて欲しい。