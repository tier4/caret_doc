# TILDE, a framework tools to detect deadline overrun

CARET can cooperate with TILDE, a framework tool to detect deadline overrun.
TILDE lets CARET trace events in user applications which cannot be traced from ROS/DDS layer.

<prettier-ignore-start>
!!! Notice
        TILDE is now under development.
<prettier-ignore-end>

Since users can implement arbitrary processing on ROS nodes, some defining intra-node path is difficult in some cases.

Examples of such difficult cases are listed as below.

- Message buffering case between subscribe and publish
- Using message filter case

the latencies can be observed in only application layer, but CARET cannot observe the events.
In such cases, data consumption in user code should be taken into account to define intra-node path. However, CARET does not observe events on user code.

To tackle this constraint, CARET is capable of utilizing events from tracepoints added by TILDE while TILDE collects application-layer events.
It is able to trace execution of callback function to consume a certain buffered message since it annotate message consumption per single message.
TILDE serves CARET such capability to trace consumption of buffered messages.

See also

- [TILDE official page](https://github.com/tier4/TILDE)
- [Node latency definition](../event_and_latency_definitions/node.md)
