# Architecture file

An architecture file is a YAML-based file which describes structure of a target application.
It contains the following information.

- Definition of the data path to be measured
- Information about the structure of the software to be measured
  - Executor information
  - Node information (e.g. node latency definition)

## File format

A sample of the architecture file is as follows.

| Key                                           | Type         | Required?                                  | Auto generate? <br> (Configuration method) | Note / Description                                 |
| --------------------------------------------- | ------------ | ------------------------------------------ | ------------------------------------------ | -------------------------------------------------- |
| named_paths                                   | List         | Yes                                        | Yes                                        | Path definitions to evaluate.                      |
| &emsp; path_name                              | String       | Yes                                        | No (Edit via Python-API)                   |                                                    |
| &emsp; node_chain                             | List         | Yes                                        | No (Edit via Python-API)                   |                                                    |
| &emsp; &emsp; node_name                       | String       | Yes                                        | No (Edit via Python-API)                   |                                                    |
| &emsp; &emsp; publish_topic_name              | String       | Required if node is not end of the path.   | No (Edit via Python-API)                   |                                                    |
| &emsp; &emsp; subscribe_topic_name            | String       | Required if node is not start of the path. | No (Edit via Python-API)                   |                                                    |
| &emsp; &emsp; publisher_construction_order    | int          | No                                         | No (Edit via Python-API)                   | Zero is used as the default value if not present.  |
| &emsp; &emsp; subscription_construction_order | int          | No                                         | No (Edit via Python-API)                   | Zero is used as the default value if not present.  |
| executors                                     | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; executor_type                          | String       | Yes                                        | Yes                                        | single_threaded_executor / multi_threaded_executor |
| &emsp; executor_name                          | String       | Yes                                        | Yes                                        |                                                    |
| &emsp; callback_group_names                   | List(String) | Yes                                        | Yes                                        |                                                    |
| nodes                                         | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; node_name                              | String       | Yes                                        | Yes                                        |                                                    |
| &emsp; callback_groups                        | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; &emsp; callback_group_type             | String       | Yes                                        | Yes                                        | mutually_exclusive / reentrant                     |
| &emsp; &emsp; callback_group_name             | String       | Yes                                        | Yes                                        |                                                    |
| &emsp; callbacks                              | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; &emsp; callback_type                   | String       | Yes                                        | Yes                                        | timer_callback / subscription_callback             |
| &emsp; &emsp; symbol                          | String       | Yes                                        | Yes                                        | symbol for callback function.                      |
| &emsp; &emsp; period_ns                       | int          | Required for timer_callback only.          | Yes                                        |                                                    |
| &emsp; &emsp; topic_name                      | String       | Required for subscription_callback only.   | Yes                                        |                                                    |
| &emsp; &emsp; construction_order              | int          | No                                         | Yes                                        | Zero is used as the default value if not present.  |
| &emsp; variable_passings                      | List         | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; callback_name_write             | String       | No                                         | No (Edit architecture file)                | default value = UNDEFINED                          |
| &emsp; &emsp; callback_name_read              | String       | No                                         | No (Edit architecture file)                | default value = UNDEFINED                          |
| &emsp; publishes                              | List         | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; topic_name                      | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; callback_names                  | List(String) | No                                         | No (Edit architecture file)                | callbacks which publish the topic.                 |
| &emsp; &emsp; construction_order              | int          | No                                         | Yes                                        | Zero is used as the default value if not present.  |
| &emsp; subscribes                             | List         | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; topic_name                      | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; callback_name                   | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; construction_order              | int          | No                                         | Yes                                        | Zero is used as the default value if not present.  |
| &emsp; message_contexts                       | List         | No                                         | Yes                                        | Field to define node latency                       |
| &emsp; &emsp; context_type                    | String       | No                                         | No (Edit architecture file)                | default value = UNDEFINED                          |
| &emsp; &emsp; subscription_topic_name         | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; publisher_topic_name            | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; publisher_construction_order    | int          | No                                         | Yes                                        | Zero is used as the default value if not present.  |
| &emsp; &emsp; subscription_construction_order | int          | No                                         | Yes                                        | Zero is used as the default value if not present.  |

## Sample

A sample of the architecture file is as follows.

```yaml
named_paths:
  - path_name: target_path
    node_chain:
      - node_name: /ping_node
        publish_topic_name: /chatter
        subscribe_topic_name: UNDEFINED
      - node_name: /pong_node
        publish_topic_name: UNDEFINED
        subscribe_topic_name: /chatter
        subscription_construction_order: 1
executors:
  - executor_type: single_threaded_executor
    executor_name: executor_0
    callback_group_names:
      - /ping_node/callback_group_0
      - /pong_node/callback_group_0
nodes:
  - node_name: /ping_node
    callback_groups:
      - callback_group_type: mutually_exclusive
        callback_group_name: /ping_node/callback_group_0
        callback_names:
          - /ping_node/callback_0
    callbacks:
      - callback_name: subscription_callback_0
        type: subscription_callback
        topic_name: /topic3
        symbol: Node::{lambda()}
      - callback_name: timer_callback_0
        type: timer_callback
        period_ns: 100000000
        symbol: Node::{lambda()}
      - callback_name: timer_callback_0
        type: timer_callback
        period_ns: 100000000
        symbol: Node::{lambda()}
        construction_order: 1
    variable_passings:
      - callback_name_write: subscription_callback_0
        callback_name_read: timer_callback_0
    publishes:
      - topic_name: /ping
        callback_names:
          - timer_callback_0
    subscribes:
      - topic_name: /pong
        callback_name: timer_callback_0
    message_contexts:
      - context_type: use_latest_message
        subscription_topic_name: /pong
        publisher_topic_name: /ping
```

## Callback identification

It's convenient for users to give a name to a callback function for its identification. However, in the context of ROS2, only an address is given to a callback.

Addresses change with each launch of an application.
This makes it difficult to handle callbacks by address when evaluating performance of them.
For example, if you want to compare the execution time of a particular callback for each launch, you have to find address to select target callbacks.

CARET helps users to give a name to a callback, but it is not directly associated with its address due to the reason as explained above.

In order to tackle the issue, CARET associates a name with an address of callback with using combination of following data.

- `node_name`
- `callback_type`
- `period_ns` / `topic_name`
- `symbol`
- `construction_order`

By using this information to match `callback_name` and callback address,
each `callback_name` will always refer to identical callbacks without being aware of callback address.
