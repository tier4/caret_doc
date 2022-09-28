# Background

CARET is capable of showing the following 4 latency:

- callback latency
- communication latency (inter-node)
- node latency (intra-node)
- path latency (intra-node & inter-node)

The following figure shows definition of node latency and path latency.

![Definition node and path latency](../imgs/path_and_node_latency.svg)

Callback latency is defined as execution time of a callback function, and can be measured with simple tracepoints. Communication latency between nodes can be defined as time from publish invoking to subscription callback invoking. Identifying callback and topic is not difficult so that their latency are calculated easily.  
However, it's difficult to define node latency and path latency mechanically. Node latency, time elapsed from input to output in a certain node, cannot be identified in ROS layer and its definition depends on patterns of application implementation. Path latency, which is defined as combination of node latency and communication latency, depends implementation patterns as well as node latency. Paths are combination of nodes which are connected via topic messages. The number of paths in a application is equaled to that of nodes combination, so that complicated and large application has large number of paths.  
To deal with such difficulty of defining node and path latency mechanically, CARET requires users to define node and path latency manually via a configurable object , called "**architecture object**".

## Architecture object

An architecture object has two sections; application structure and latency definition. Application structure section describes components of a target application and their connections, represented as executors, nodes, callback groups, callbacks, topics, and timers. CARET can load an architecture object including only application structure section from [CTF](https://diamon.org/ctf/)-based recorded data. The application structure section remains same unless the structure is changed or component is renamed.

On the other hand, latency definition section in the template architecture file is empty just before users add any definition. Users are expected to add definitions of node latency and path latency in the template file. CARET helps users to add definition of path latency with Python API. However, users have to add definition of node latency manually with editing the architecture file.

The following sections explains how to create an architecture file and add latency definition.