# Preparation for measuring latency of callback-chains

CARET is capable of measuring latency of not only callback functions and topic communications, but also node chains.
Specifying a callback function as target is very simple because name is corresponding to the callback function. Topic communication is specified by a topic name, a source node, and a destination node. However, specifying a node chain is more difficult than them. A node chain is defined as a combination of callback functions and topic communications, and often called as "path". Though node chain is regarded as DAG (Direct Acyclic Graph), a node chain may have loops and forks. Loops and forks make a node chain make complicated. Additionally, a large system tends to have many candidates of node chains because it is constructed of many callback functions which are communicated via many topics.

## Overview of an architecture file

To deal with such difficulty of specifying node chains, CARET serves configuration for users to choose target node chains. The configuration is defined in a yaml file, which is called "**architecture file**". An architecture file describes a target application's structure. CARET can create an architecture file for minimum description automatically with using measured data, but it does not have target node chains. You will add description for target node chains to a generated architecture file.

The rest of this section will explain more details of an architecture file.

## What to describe in an architecture file

An architecture file is a yaml-based file which has description of an application measured by CARET. In particular, an architecture file includes information representing the application structure, such as executors, nodes, callback groups, callbacks, topics, timers and so on. It can be used for another trial measurement because such information shall be same unless the application is changed.

---

User have to add the following description to architecture file.

1. Target node chains and its structure
2. Intra-communication of topic in a single node to calculate latency of the node

![Target node chains and path latency calculate](../imgs/path_and_node_latency.svg)

CARET serves Python API to make 1. easier, but you have to edit architecture file for 2.

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

## How to specify a target node chain

1. Load the yaml-based architecture file as below

   ```python
   from caret_analyze import Architecture, check_procedure
   arch = Architecture('yaml', './architecture.yaml')
   ```

2. Specify source node and destination node in a node chain

   `arch.search_paths` extract all candidates of the node chain.

   ```python
   paths = arch.search_paths(
   '/sensor_dummy_node', # source node
   '/actuator_dummy_node') # destination node
   ```

   If a target application is large and complicated, `arch.search_paths` method may consume time more than 1 minute.
   For decreasing consumed time, you can ignore nodes and topics and specify depth of search. Refer to [パスの探索方法](../supplements/how_to_search_path.md) for more details.

3. Check the node chain as you expected

   You will find multiple candidates of the node chain. You can check which candidate is expected as target. The following code is an example for users to check

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
   # in /message_driven_node
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

   `path.verify()` tells you that there is no undefined node latency in the path (node chain).

   ```python
   from caret_analyze import Architecture

   arch = Architecture('yaml', './architecture.yaml')
   path = arch.get_path('target_path')
   path.verify()
   ```

   If `path.verify()` returns `True`, CARET can calculate latency of the node chain. Otherwise, there is any lack of definition to calculate latency.

<prettier-ignore-start>
!!! todo
        We'll provide the sample architecture file here, but it's not ready. Sorry for inconvenience.
<prettier-ignore-end>
