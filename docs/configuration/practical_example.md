# Practical example of configuration

The previous sections have explained what to do in configuration. This section explains you a practical example to let you understand the flow of configuration.

This section demonstrates creation of an architecture file for `CARET_demos` on Jupyter Notebook. Defining inter-node and intra-node data path is also demonstrated. The following three steps are explained.

1. Load an Architecture object on Jupyter Notebook
2. Define an inter-node data path
3. Define an intra-node data path

The architecture file that will be created in the example is provided [here](https://raw.githubusercontent.com/tier4/CARET_demos/main/samples/end_to_end_sample/architecture.yaml).

## Load an Architecture object on Jupyter Notebook

Load an architecture object from recorded data, as explained section of [Load and save](./load_and_save.md)

1. Launch Jupyter Notebook (Jupyter Lab)

   ```bash
   mkdir -p ~/ros2_ws/evaluate && cd ~/ros2_ws/evaluate

   source ~/ros2_caret_ws/install/setup.bash
   jupyter-lab
   ```

2. Generate an architecture file from recorded data as below

   ```python
   from caret_analyze import Architecture

   # Read description of application's architecture from recorded data
   #
   arch = Architecture('lttng', './e2e_sample')

   # Save description as an architecture file
   arch.export('architecture.yaml')

   # Check if the architecture file is created
   ! readlink -f ./architecture.yaml
   # /home/user/ros2_caret_ws/eval/architecture.yaml
   ```

## Define an inter-node data path

Define an inter-node data path on the loaded architecture object as section of [Define inter-node data path](./inter_node_data_path.md) explained

1. Load the yaml-based architecture file as below

   ```python
   from caret_analyze import Architecture, check_procedure
   arch = Architecture('yaml', './architecture.yaml')
   ```

2. Choose source node and destination node in a path

   `arch.search_paths` extract all candidates of the path

   ```python
   paths = arch.search_paths(
   '/sensor_dummy_node', # source node
   '/actuator_dummy_node') # destination node
   ```

   If a target application is large and complicated, `arch.search_paths` method may consume time more than 1 minute.
   For decreasing consumed time, you can ignore nodes and topics and specify depth of search. Refer to [Define inter-node data path](../configuration/inter_node_data_path.md) for more details.

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
   arch.add_path('target', path)
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

## Define intra-node data path

Define an intra-node data path on the loaded architecture object as section of [Define intra-node data path](./intra_node_data_path.md) explained

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
   # in /timer_driven_node
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
