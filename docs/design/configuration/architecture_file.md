# Architecture file

An architecture file is a YAML-based file which describes structure of a target application.
It contains the following information.

- Definition of the data path to be measured
- Information about the structure of the software to be measured
  - Executor information
  - Node information (e.g. node latency definition)

## File format

A sample of the architecture file is as follows.

| Key                                   | Type         | Required?                                  | Auto generate? <br> (Configuration method) | Note / Description                                 |
| ------------------------------------- | ------------ | ------------------------------------------ | ------------------------------------------ | -------------------------------------------------- |
| named_paths                           | List         | Yes                                        | Yes                                        | Path definitions to evaluate.                      |
| &emsp; path_name                      | String       | Yes                                        | No (Edit via Python-API)                   |                                                    |
| &emsp; node_chain                     | List         | Yes                                        | No (Edit via Python-API)                   |                                                    |
| &emsp; &emsp; node_name               | String       | Yes                                        | No (Edit via Python-API)                   |                                                    |
| &emsp; &emsp; publish_topic_name      | String       | Required if node is not end of the path.   | No (Edit via Python-API)                   |                                                    |
| &emsp; &emsp; subscribe_topic_name    | String       | Required if node is not start of the path. | No (Edit via Python-API)                   |                                                    |
| executors                             | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; executor_type                  | String       | Yes                                        | Yes                                        | single_threaded_executor / multi_threaded_executor |
| &emsp; executor_name                  | String       | Yes                                        | Yes                                        |                                                    |
| &emsp; callback_group_names           | List(String) | Yes                                        | Yes                                        |                                                    |
| nodes                                 | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; node_name                      | String       | Yes                                        | Yes                                        |                                                    |
| &emsp; callback_groups                | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; &emsp; callback_group_type     | String       | Yes                                        | Yes                                        | mutually_exclusive / reentrant                     |
| &emsp; &emsp; callback_group_name     | String       | Yes                                        | Yes                                        |                                                    |
| &emsp; callbacks                      | List         | Yes                                        | Yes                                        |                                                    |
| &emsp; &emsp; callback_type           | String       | Yes                                        | Yes                                        | timer_callback / subscription_callback             |
| &emsp; &emsp; symbol                  | String       | Yes                                        | Yes                                        | symbol for callback function.                      |
| &emsp; &emsp; period_ns               | int          | Required for timer_callback only.          | Yes                                        |                                                    |
| &emsp; &emsp; topic_name              | String       | Required for subscription_callback only.   | Yes                                        |                                                    |
| &emsp; variable_passings              | List         | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; callback_name_write     | String       | No                                         | No (Edit architecture file)                | default value = UNDEFINED                          |
| &emsp; &emsp; callback_name_read      | String       | No                                         | No (Edit architecture file)                | default value = UNDEFINED                          |
| &emsp; publishes                      | List         | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; topic_name              | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; callback_names          | List(String) | No                                         | No (Edit architecture file)                | callbacks which publish the topic.                 |
| &emsp; subscribes                     | List         | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; topic_name              | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; callback_name           | String       | No                                         | Yes                                        |                                                    |
| &emsp; message_contexts               | List         | No                                         | Yes                                        | Field to define node latency                       |
| &emsp; &emsp; context_type            | String       | No                                         | No (Edit architecture file)                | default value = UNDEFINED                          |
| &emsp; &emsp; subscription_topic_name | String       | No                                         | Yes                                        |                                                    |
| &emsp; &emsp; publisher_topic_name    | String       | No                                         | Yes                                        |                                                    |

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
