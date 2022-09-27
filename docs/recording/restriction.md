# CARET Restrictions

- CARET identifies callback functions using the following information
  - Timer callback functions: timer period
  - Subscription callback functinos: topic name
- Therefore, if there are several callbacks whose parameters are the same in a node, CARET cannot analyze the node correctly. As a result, those nodes are ignored
- In case you see warnings and the relevant node need to be analyzed, please modify the code

## Timer Callbacks

### About

### Error message

### How to fix

## Subscription Callbacks

### About

### Error message

### How to fix

## Multiple executors using the same callback group

```txt
Multiple executors using the same callback group were detected.The last executor will be used. executor address: [94351911253008, 140728133491392]
```

A callback group is added to several executors

```cpp
exec1.add_node(node);
exec2.add_node(node)
```

## Failed to identify subscription. Several candidates were found

```txt
Failed to identify subscription. Several candidates were found. node_name: /abc, topic_name: /xyz
```

A node creates several subscription callback functions with the same topic name.

```cpp
node_abc.create_subscription("/xyz", callback1);
node_abc.create_subscription("/xyz", callback2);
```

## Failed to find callback group

```txt
Failed to find callback group. callback_group_id=callback_group_93827282066928
```

CARET failed to bind callback groups, callbacks and executors. It mainly because of "service". "Service" is not supported by CARET.
