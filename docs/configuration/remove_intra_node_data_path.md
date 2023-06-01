# How to define intra-node data path

In the previous section, you learned how to define inter-node data path.
CARET served Python-based APIs to define data path, and the API could rewrite architecture files.

This section will explain how to remove intra-node data paths defined by the API in the previous section.

### Python API

CARET serves Python-based APIs to remove intra-node data path. The example of previous section is used for explanation.

Incidentally, essential description is extracted in the following snippet, but you will confront with busy YAML file actually rather than the sample.

All of the following code snippets can be executed after the `Architecture('type', 'file_path')` method loads an architecture object.
The architecture object whose sub-objects are renamed is saved to a file as explained in [the previous page](./load_and_save.md#save).

#### `use_latest_message`

Using the following Python API to remove intra-node data path of `use_latest_message` on an architecture file.

You can remove callback to publisher with `insert_publisher_callback` function in `Architecture` class.
As arguments, the target node name, publishing topic name and publisher callback name must be specified.

Then, you can update `context_types` to `UNDEFINED` between targeted subscription and publisher with `update_message_context` function in `Architecture` class.
As arguments, the target node name, subscription topic name and publisher topic name must be specified.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.remove_publisher_callback('/pong_node', '/pong', 'timer_callback_1')
arch.update_message_context('/pong_node', '/ping', '/pong', 'UNDEFINED')
```

As a result of these processes, data path of `use_latest_message` is removed.
Incidentally, the output of the architecture file is updated as follows.

```yml
- node_name: /pong_node
  callbacks:
    - callback_name: subscription_callback_0
    - callback_name: timer_callback_1
  publishes:
    - topic_name: /pong
      callback_names:
        - UNDEFINED # updated
  subscribes:
    - topic_name: /ping
      callback_name: subscription_callback_0
  message_contexts:
    - context_type: UNDEFINED # updated
      subscription_topic_name: /ping
      publisher_topic_name: /pong
```

#### `callback_chain`

Using the following Python API to remove intra-node data path of `callback_chain` on an architecture file.

You can remove callback to publisher with `insert_publisher_callback` function in `Architecture` class.
As arguments, the target node name, publishing topic name and publisher callback name must be specified.

Then, you can remove variable passing with `insert_variable_passing` function in `Architecture` class.
As arguments, the target node name, write callback name and read callback name must be specified.

Finally, you can update `context_types` to `UNDEFINED` between targeted subscription and publisher with `update_message_context` function in `Architecture` class.
As arguments, the target node name, subscription topic name and publisher topic name must be specified.

```python
arch.remove_publisher_callback('/pong_node', '/pong', 'timer_callback_1')
arch.remove_variable_passing('/pong_node', 'subscription_callback_0', 'timer_callback_1')
arch.update_message_context('/pong_node', '/ping', '/pong', 'UNDEFINED')
```

As a note, `update_message_context` should be done after `remove_publisher_callback` and `remove_variable_passing`, as CARET complements `UNDEFINED` context type in `callback_chain` context type when `variable_passings` and `publishers` are present.

As a result of these processes, data path of `callback_chain` is removed.
Incidentally, the output of the architecture file is updated as follows.

```yml
- node_name: /pong_node
  callbacks:
    - callback_name: subscription_callback_0
    - callback_name: timer_callback_1
  variable_passings:
    - callback_name_write: UNDEFINED # updated
      callback_name_read: UNDEFINED # updated
  publishes:
    - topic_name: /ping
      callback_names:
        - UNDEFINED # updated
  subscribes:
    - topic_name: /pong
      callback_name: UNDEFINED
  message_contexts:
    - context_type: UNDEFINED # updated
      subscription_topic_name: /pong
      publisher_topic_name: /ping
```
