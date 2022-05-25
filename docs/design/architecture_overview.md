# Architecture overview

This section explains an overview of software architecture of CARET.

![architecture](../imgs/architecture.png)

CARET collects data, which include timestamps, from tracepoints embedded in user application, ROS 2 and DDS. The data set is stored into a storage as "Trace Data".

A set of trace data is divided into two sections by CARET_analyze package after loading trace data; Architecture and Runtime Data. Architecture object includes description of target application's structure. This object can be reused unless structure of the target application or name of the components is changed. Runtime Data object has data sampled during execution of the target application. The sampled data includes timestamps, whose value are different per execution, obtained from tracepoints.

Architecture object and Runtime Data object are implemented as Python-based classes. The structure of their classes are designed based on the structure of ROS applications which are constructed of executors, nodes, callback functions, and topic messages. ROS-based structure makes CARET's API friendly for ROS users. They are able to find target nodes, topic messages, or executors if they know their application structure.

Architecture object serves APIs to search node chains and define node latency as mentioned in [tutorial/architecture file section](../tutorials/configuration.md). Architecture object is reusable after it is saved as a YAML-based file called "architecture file".  
Runtime Data object serves APIs to retrieve `pandas.DataFrame`-based objects including callback latency or communication. Users can analyze temporal aspects of their applications, with visualization, as they expect. APIs for visualization are also served by CARET_analyze which plays main role to analyze trace data.

## Cooperation with TILDE, a framework tools to detect deadline overrun

CARET can cooperate with TILDE, a framework tool to detect deadline overrun. TILDE lets CARET trace events in user applications which cannot be traced from ROS/DDS layer.

One example of the events is consumption of message buffered in nodes. The events can be observed in only application layer, but CARET cannot observe the events. On the other hand, TILDE can trace application-layer events. It is able to trace execution of callback function to consume a certain buffered message since it annotate message consumption per single message. TILDE serves CARET this capability to trace consumption of buffered message.

<prettier-ignore-start>
!!! todo
        TILDE is now under development. Please wait for more details.
<prettier-ignore-end>
