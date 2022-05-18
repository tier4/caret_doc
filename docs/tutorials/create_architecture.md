# Preparation for measuring node and path latency

CARET can observe as following:

- callback latency
- communication latency (inter-node)
- node latency (intra-node)
- path latency (intra-node & inter-node)

The following figure shows definition of node latency and path latency.

![Definition node and path latency](../imgs/path_and_node_latency.svg)

Callback latency can be defined as execution time of a callback function and can be measured with simple tracepoints. Communication latency between nodes can be defined as time from publish invoking to subscription callback invoking. Identifying callback and topic is not difficult so that their latency are calculated easily.  
However, it's difficult to define node latency and path latency mechanically. Node latency, time elapsed from input to output in a certain node, cannot be identified in ROS layer and its definition depends on patterns of application implementation. Path latency, which is defined as combination of node latency and communication latency, depends implementation patterns as well as node latency. Paths are combination of nodes which are connected via topic messages. The number of paths in a application is equaled to that of nodes combination, so that complicated and large application has large number of paths.  
To deal with such difficulty of defining node and path latency mechanically, CARET requires users to define node and path latency manually via a configuration file, called "**architecture file**".

## Overview of an architecture file

An architecture file has two sections; application structure and latency definition. Application structure section describes components of a target application and their connections, represented as executors, nodes, callback groups, callbacks, topics, and timers. CARET can create a template architecture file including only application structure section. The application structure section remains same unless the structure is changed or component is renamed.

On the other hand, latency definition section in the template architecture file is empty just before users add any definition. Users are expected to add definitions of node latency and path latency in the template file. CARET helps users to add definition of path latency with Python API. However, users have to add definition of node latency manually with editing the architecture file.

The following sections explains how to create an architecture file and add latency definition.

## How to generate an architecture file

This section explains how to generate an architecture file which has minimum description.

1. Launch Jupyter Notebook (Jupyter Lab)

   ```bash
   mkdir -p ~/ros2_ws/evaluate && cd ~/ros2_ws/evaluate

   source ~/ros2_caret_ws/install/setup.bash
   jupyter-lab
   ```

2. Generate an architecture file from measured data as below

   ```python
   from caret_analyze import Architecture

   # Read description of application's architecture from measured data
   arch = Architecture('lttng', './e2e_sample')

   # Save description as an architecture file
   arch.export('architecture.yaml')

   # Check if the architecture file is created
   ! readlink -f ./architecture.yaml
   # /home/user/ros2_caret_ws/eval/architecture.yaml
   ```

## How to specify a target path

1. Load the yaml-based architecture file as below

   ```python
   from caret_analyze import Architecture, check_procedure
   arch = Architecture('yaml', './architecture.yaml')
   ```

2. Specify source node and destination node in a path

   `arch.search_paths` extract all candidates of the path

   ```python
   paths = arch.search_paths(
   '/sensor_dummy_node', # source node
   '/actuator_dummy_node') # destination node
   ```

   If a target application is large and complicated, `arch.search_paths` method may consume time more than 1 minute.
   For decreasing consumed time, you can ignore nodes and topics and specify depth of search. Refer to [パスの探索方法](../supplements/how_to_search_path.md) for more details.

3. Check the path as you expected

   You will find multiple candidates of the path. You can check which candidate is expected as target. The following code is an example for users to check

   ```python
   path = paths[0]
   path.summary.pprint()

   ---Output text as below---

   path:
     - message_context: null # for definition of node latency
       node: /sensor_dummy_node
     - topic: /topic1
     - message_context:
         publisher_topic_name: /topic2
         subscription_topic_name: /topic1
         type: callback_chain
       node: /filter_node
     - topic: /topic2
     - message_context: null
       node: /message_driven_node
     - topic: /topic3
     - message_context: null
       node: /timer_driven_node
     - topic: /topic4
     - message_context: null
       node: /actuator_dummy_node
   ```

4. Give a name to selected path and update architecture file

   ```python
   arch.add_path('target_path', path)
   arch.export('./architecture.yaml', force=True)
   ```

   The updated architecture file describes the path named as `target_path`.

   ```yaml
   named_paths:
     - path_name: target_path
       node_chain:
         - node_name: /sensor_dummy_node
           publish_topic_name: /topic1
           subscribe_topic_name: UNDEFINED
         - node_name: /filter_node
           publish_topic_name: /topic2
           subscribe_topic_name: /topic1
         - node_name: /message_driven_node
           publish_topic_name: /topic3
           subscribe_topic_name: /topic2
         - node_name: /timer_driven_node
           publish_topic_name: /topic4
           subscribe_topic_name: /topic3
         - node_name: /actuator_dummy_node
           publish_topic_name: UNDEFINED
           subscribe_topic_name: /topic4
   ```

## How to define latency of a single node

Latency of a single node, so called "node latency", is defined as elapsed time between 1. starting time and 2. publishing time as below.

1. starting time when node subscribes topic message and invokes a corresponding callback function
2. publish time: when node publishes topic message

Definition of node latency depends on implementation pattern. Some nodes subscribe input messages and invoke callback function where they publish output messages. These nodes has direct relationship between input and output. Other nodes subscribe input messages and invoke callback functions where they buffer them, and invoke different callback functions consume input messages and publish output message. In the latter cases, relationship of input and output is indirect, and intra-node communication is performed with using multiple callback functions. [message_filters](http://wiki.ros.org/message_filters) is another cause to increase the number of implementation patterns.

Therefore, CARET has to deal with several types of node implementation to measure node latency. CARET serve a function to define node latency with an architecture file. An architecture file has an item of **mesasge_context**, which indicates relation between input message and output message. This item should be defined by users as below.

1. Check which node latency should be configured

   `path.verify()` method, as shown in the following example, tells you which node latency should be defined.

   ```python
   from caret_analyze import Architecture

   arch = Architecture('yaml', './architecture.yaml')
   path = arch.get_path('target_path')
   path.verify()

   ---Output text as below---
   WARNING : 2021-12-20 19:14:03 | Detected "message_contest is None". Correct these node_path definitions.
   To see node definition and procedure,execute :
   >> check_procedure('yaml', '/path/to/yaml', arch, '/message_driven_node')
   message_context: null
   node: /message_driven_node
   publish_topic_name: /topic3
   subscribe_topic_name: /topic2

   WARNING : 2021-12-20 19:14:03 | Detected "message_contest is None". Correct these node_path definitions.
   To see node definition and procedure,execute :
   >> check_procedure('yaml', '/path/to/yaml', arch, '/timer_driven_node')
   message_context: null
   node: /timer_driven_node
   publish_topic_name: /topic4
   subscribe_topic_name: /topic3
   ```

   In the example, `path.verify()` tells you two nodes have undefined relationships of input and output.

   - input `/topic2` and output `/topic3` in node `/message_driven_node`
   - input `/topic3` and output `/topic4` in node `/timer_driven_node`

   Their relationships must be explicit with corresponding message_context items in the architecture file.

2. Define relationship between input and output

   You have to change message_contexts items as below for the sample.

   ```yaml
   # in /timer_driven_node
   message_contexts:
     - context_type: use_latest_message # changed from 'UNDEFINED' to 'use_latest_message'
       subscription_topic_name: /topic2
       publisher_topic_name: /topic3
   ```

   ```yaml
   # in /message_driven_node
     message_contexts:　
     - context_type: use_latest_message # changed from 'UNDEFINED' to 'use_latest_message'
       subscription_topic_name: /topic3
       publisher_topic_name: /topic4

   ```

3. Check if node latency is defined

   `path.verify()` tells you that there is no undefined node latency in the path.

   ```python
   from caret_analyze import Architecture

   arch = Architecture('yaml', './architecture.yaml')
   path = arch.get_path('target_path')
   path.verify()
   ```

   If `path.verify()` returns `True`, CARET can calculate latency of the path. Otherwise, there is any lack of definition to calculate latency.

<prettier-ignore-start>
!!! todo
        We'll provide the sample architecture file here, but it's not ready. Sorry for inconvenience.
<prettier-ignore-end>
