# Visualization

This chapter explains how to visualize trace data as you expects. [Gallery](../gallery.md) section shows which type of visualization is supported by CARET. CARET serves API to output
We recommend to read [Basic APIs Concept (Visualization)](./basic_api_concept.md) at first.

## Basic APIs Concept

- [Gallery](../gallery.md)
- [Basic APIs Concept (Visualization)](./basic_api_concept.md)

## CARET API List

### Visualization APIs

#### Callback

- [callback_sched](./callback/callback_scheduling_visualization.md)
  - Visualize callback scheduling
- [create_callback_frequency_plot](./callback/index.md#execution-frequency)
- [create_callback_period_plot](./callback/index.md#period)
- [create_callback_latency_plot](./callback/index.md#latency)

#### Communication

- [create_communication_latency_plot](./communication/index.md#latency)
- [create_communication_frequency_plot](./communication/index.md#frequency)
- [create_communication_period_plot](./communication/index.md#period)
- [create_publish_subscription_period_plot](./communication/publish_subscription.md#period)
- [create_publish_subscription_frequency_plot](./communication/publish_subscription.md#frequency)

#### Path

- [message_flow](./path/message_flow.md)
  - Visualize the message flow of the target path
- [create_response_time_histogram_plot](./path/response_time.md)
- [chain_latency](./path/chain_latency.md)

### Helper APIs

CARET provides some APIs which can help users to focus on their respective interest.

- [LTTngEventFilter](./filter/lttng_event_filter.md)
- [Wildcards for get_callbacks()](./search/wildcards_for_get_callbacks.md)

<prettier-ignore-start>
!!!info
    [CARET analyze API document](https://tier4.github.io/CARET_analyze/) describes the parameters and returns of APIs.
<prettier-ignore-end>
