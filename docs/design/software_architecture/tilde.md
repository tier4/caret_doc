# TILDE, a framework tools to detect deadline overrun

CARET can cooperate with TILDE, a framework tool to detect deadline overrun.
TILDE lets CARET trace events in user applications which cannot be traced from ROS/DDS layer.

<prettier-ignore-start>
!!! Notice
        TILDE is now under development.
<prettier-ignore-end>

Since ROS nodes can implement arbitrary processing, some nodes are difficult to calculate node latency.

Examples of implementations for which node latency is difficult to calculate are as follows.

- Message buffering case between subscribe and publish
- Using message filter case

the latencies can be observed in only application layer, but CARET cannot observe the events.

On the other hand, TILDE can trace application-layer events.
It is able to trace execution of callback function to consume a certain buffered message since it annotate message consumption per single message.

TILDE serves CARET this capability to trace consumption of buffered messages.

See also

- [TILDE official page](https://github.com/tier4/TILDE)
- [Node latency definition](../event_and_latency_definitions/node.md)
