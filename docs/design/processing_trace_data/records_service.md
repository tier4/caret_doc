# Records service

The Records object holds time-series data such as message flow in a table as shown below.

| Start timestamp | ... | End timestamp |
| --------------- | --- | ------------- |
| 0.0             | ... | 0.1           |
| 1.0             | ... | 1.1           |
| 2.0             | ... | NaN           |
| 3.0             | ... | 2.1           |
| ...             | ... | ...           |

The start_timestamp column contains the system time at the starting point.
The end_timestamp column contains the system time at the end point.
Both columns indicate the system time, and if there is no corresponding value for start, the value is NaN.

For the intermediate columns, the time at the intermediate point between start and end is represented.

This table representation can be used for various measurement targets such as callbacks, nodes, and paths.

The rows of the table are visualized as lines in a message flow diagram.

This section describes the process o
The class that reads TraceData is the Records object.

- [Records](./records.md)

The following are processing using Records object.

- [Period](./records_service.md#period)
- [Frequency](./records_service.md#frequency)
- [Latency](./records_service.md#latency)
- [Response time](./records_service.md#response-time)

See also

- [Latency definition](../latency_definitions/index.md)

See also

- [Records](./records.md)

## Period

The period is defined as the time difference between the columns of interest.

$$
    period_n = t_{n} -t_{n-1}
$$

### Example

Input

| Start timestamp |
| --------------- |
| 0.0             |
| 1.0             |
| 2.0             |
| 3.0             |
| ...             |

Output

| Timestamp | Period |
| --------- | ------ |
| 0.0       | 1.0    |
| 1.0       | 1.0    |
| 2.0       | 1.0    |
| ...       | ...    |

See also

- [API Reference | Period](https://tier4.github.io/CARET_analyze/latest/record/#caret_analyze.record.Period)

## Frequency

Frequency is defined as the number of events that occur in one second.

### Example

Input

| Start timestamp |
| --------------- |
| 0.0             |
| 0.1             |
| 0.5             |
| 1.2             |
| 1.3             |
| 2.3             |
| ...             |

Output

| Timestamp | Period |
| --------- | ------ |
| 0.0       | 3.0    |
| 1.0       | 2.0    |
| ...       | ...    |

## Latency

Latency is defined as the time difference between the two columns of interest.

$$
latency_n = t^{end}_{n} - t^{start}_{n}
$$

### Example

Input

| Start timestamp | End timestamp |
| --------------- | ------------- |
| 0.0             | 0.1           |
| 1.0             | 1.1           |
| 2.0             | NaN           |
| 3.0             | 3.1           |
| ...             | ...           |

Output

| Start timestamp | Latency |
| --------------- | ------- |
| 0.0             | 0.1     |
| 1.0             | 0.1     |
| 3.0             | 0.1     |
| ...             | ...     |

## Response Time

Response time is the amount of time it takes for a system to respond to an input.

As shown above, latency can be calculated if a table can be constructed.
However, the calculated latency is not suitable for evaluating response time.
For example, if a sensor is driven at 10 Hz, a latency of up to 100 ms should be considered.

CARET defines latency as the best-case response time and worst-case response time as the value that includes the delay due to cycles, as a more comprehensive approach.

### Example

Input

| Start timestamp | End timestamp |
| --------------- | ------------- |
| 0.0             | 0.1           |
| 1.0             | 1.1           |
| 2.0             | NaN           |
| 3.0             | 3.2           |
| 4.0             | 4.3           |
| ...             | ...           |

Intermediate  
Create intermediate data for when the interval in Start timestamp [0.0, 4.0] maps to End timestamp.

| Start timestamp | End timestamp |
| --------------- | ------------- |
| [0.0, 1.0)      | 1.1           |
| [1.0, 3.0)      | 3.2           |
| [3.0, 4.0)      | 4.3           |

Output

| Start timestamp | Best-case response time | Worst-case response time |
| --------------- | ----------------------- | ------------------------ |
| 1.0             | 0.1 (1.1 - 1.0)         | 1.1 (1.1 - 0.0)          |
| 3.0             | 0.2 (3.2 - 3.0)         | 2.3 (3.2 - 1.0)          |
| ...             | ...                     | ...                      |

Note that Best-case response time has the same definition as Latency.
Worst-case response time also counts as response time in the case of a drop.

### Visualize response time

![Latency vs Response time](../../imgs/latency_vs_response_time.drawio.png)

Cases like the one shown above include

- Cases where multiple latencies are defined for a single output
- Message dropping
- Crossing
- Branching

Some of these can be pessimistically large when it comes to latency.

The diagram above describes the table as follows

| Start timestamp | End timestamp |
| --------------- | ------------- |
| 0.0             | 2.0           |
| 0.5             | 2.5           |
| 2.0             | 3.5           |
| 3.0             | 3.5           |
| 4.0             | 5.0           |
| 4.5             | 7.0           |
| 5.5             | 6.0           |
| 5.5             | 6.5           |

After leaving only the best-case flows as shown in the figure below,
the best-case is calculated as the latency of the line and the worst-case as the latency calculated from the previous input.

The response time after processing is shown below.

| Start timestamp | Min response time | Max response time |
| --------------- | ----------------- | ----------------- |
| 0.0             | 2.0               | 2.5               |
| 0.5             | 0.5               | 3.0               |
| 3.0             | 1.0               | 2.0               |
| 4.0             | 0.5               | 2.0               |

See also

- [FAQ | How response time is calculated?](../../faq/index.md#how-response-time-is-calculated)
