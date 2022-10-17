# Visualization

## Basic APIs Concept

- [Basic APIs Concept](./concept/basic_api_concept.md)
- [Gallery](../gallery.md)

## Provided APIs

### APIs in Plot Class

- [create_callback_frequency_plot](./visualization_api/callback_information#execution-frequency)
- [create_callback_period_plot](./visualization_api/callback_information#period)
- [create_callback_latency_plot](./visualization_api/callback_information#latency)
- [create_publish_subscription_period_plot](./visualization_api/pub_sub_information)
- [create_publish_subscription_frequency_plot](./visualization_api/pub_sub_information)
- [create_communication_latency_plot](./visualization_api/communication_information)
- [create_communication_frequency_plot](./visualization_api/communication_information)
- [create_communication_period_plot](./visualization_api/communication_information)
- [create_response_time_histogram_plot](./visualization_api/response_time)

### APIs in other than Plot Class

Note: These APIs will be implemented to Plot Class.

- [message_flow](./visualization_api/message_flow.md)
- [callback_sched](./visualization_api/callback_scheduling_visualization.md)

### Investigation APIs

CARET provides some APIs to investigate more detail.

- [LTTngEventFilter](./investigation_api/lttng_event_filter.md)
- [get_callbacks](./investigation_api/investigate_behavior.md)

Note: Investigation APIs are written in [CARET analyze API document](https://tier4.github.io/CARET_analyze/).
