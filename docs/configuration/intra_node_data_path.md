# How to define intra-node data path

In the previous section, you learned how to define inter-node data path. However, response time for path cannot be calculated with only inter-node data path definition. As you will see in the chapter, you have to define intra-node data path.

CARET assume that intra-node data path is duration from topic subscription to topic publish in a node. This is not defined simply because the definition depends on implementation.

This section will explain how to find which intra-node data path you have to define, and how to define it.

## Find which intra-node data path to define

Before defining intra-node data path, you have to check whether latency of the target path is calculable. This can be checked with `Path.verify()` method as the following sample code shows.

```python
arch = Architecture('yaml', '/path/to/architecture.yaml')

path = arch.get_path('target_path')
path.verify()
```

Here, `path.verify()` returns `True` if path latency for `path` is calculable. Otherwise, you will confront with warning messages as shown below.

```python
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

On the sample warning message, `/message_driven_node` and `/timer_driven_node` have undefined intra-node data path. You will add the intra-node data path in next step.

## How to add definition intra-node data path to Architecture object

### Python API

### Architecture file editing

## Limitation
