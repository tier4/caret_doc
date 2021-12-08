#

DDS レイテンシを算出するにあたって

CycloneDDS は on_data_available を実行していない。

そのため、フラグを操作し、強制的に on_data_available を実行させるようにした。

ただし、on_data_available は rmw_wait の起床とは別物なので、

on_data_available -> rmw_wait 起床とはならないケースがある。

従って、cycloneDDS の DDS レイテンシは参考程度に考えて欲しい。
