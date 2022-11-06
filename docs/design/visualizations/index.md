# Visualizations

CARET is a tool for performance evaluation.

For evaluation, quantification and visualization would be sufficient.  
For analysis, on the other hand, various factors are involved in behavior and performance, so it is necessary to investigate from multiple perspectives.

CARET has multiple visualizations for evaluation and analysis APIs that are readily available.  
Also, it's also possible to directly acquire and evaluate the data on your own (see [Processing trace data](../processing_trace_data/index.md) for detail).

## Policy

### Evaluation flow

CARET has a large amount of information that can be obtained, and it is difficult to evaluate large-scale applications simply by displaying the recorded trace data in time series.
So, it is important to focus on evaluation targets with appropriate granularity according to the purpose of evaluation.

The following diagram shows the evaluation flow and analysis by adjusting the granularity.

![Analysis flow](../../imgs/analysis_flow.drawio.png)

Here, the horizontal axis represents the visualization granularity and the vertical axis represents the processing granularity.  
CARET leads from issue detection to cause identification by changing the processing and visualization granularity from the rough granularity in the upper right to the fine granularity in the lower left.

1. Detect issues: Detects performance issues on the target system.
2. Identify issues: Identify the bottleneck that is causing the issue.
3. Identify causes: Identifies the cause of the bottleneck.

Processing granularity on the horizontal axis and visualization granularity on the vertical axis are explained in the following section.

### Visualization granularity

![Visualization Granularity](../../imgs/visualization_granularity.drawio.png)

The granularity of visualization can be ordered in coarse order as follows

- Statistic
- Histogram
- Heatmap
- Bar graph / Line graph
- Time-series trace data

The coarser granularity, the more time information is aggregated to evaluate the measurement as a whole.  
The most granular statistics are suitable for regression testing.

On the other hand, the finer granularity, the more detailed information is expressed, so it is more suitable for analysis. evaluation
Latency and response time are some of the indicators to evaluate performance.  
The most detailed information is a time-series graph of each trace data.

### Processing granularity

![Processing Granularity](../../imgs/processing_granularity.drawio.png)

The processing granularity can be ordered in coarse order as follows.

- path
- node / communication
- callbackCurrent CARET does not support measurement of arbitrary functions or DDS enqueue/dequeue.
- function

Path is suitable for evaluation of system performance, while node and callback are suitable for evaluation of functional performance.

<prettier-ignore-start>
!!! Notice
        Currently CARET does not support measurement of arbitrary functions or DDS enqueue/dequeue or system calls.
<prettier-ignore-end>

See also

- [Latency definition](../latency_definitions/index.md)

### Time domain and Frequency domain

We described behavior and indicator in terms of visualization granularity, but there are multiple indicators, such as latency and frequency.  
When considering which indicator to use, it is useful to consider the time domain indicators and frequency domain indicators.

- Time Domain Indicators (ex: callback execution time [s])
- Frequency Domain Indicators (ex: topic frequency [Hz])

Both have their pros and cons.

|           | Time Domain Indicators                   | Frequency Domain Indicators                   |
| --------- | ---------------------------------------- | --------------------------------------------- |
| Indicator | Latency, Response time                   | Frequency (, Period)                          |
| Pros      | Easy to compare with system requirements | No need to define latency or path             |
| Cons      | Need to define latency or path           | Difficult to compare with system requirements |

In the table above, period is an indicator that expresses the time interval between each time, so it is classified in the frequency domain.

See also

- [Records service](../processing_trace_data/records_service.md)
