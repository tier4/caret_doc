# How to get differences of architectures

An architecture object has structure of a target application. It is useful when you want to find the structure. In some cases, you want to check difference between the current version and the previous one of the target application. The difference tells you which executor, node, callback, or topic is updated. CARET serves the function to get difference between two architecture objects. This function is called `diff` function in the following.

The `diff` function can be used to find the difference between two architecture objects. Specifically, the `diff` functions can find data that exist only in one of the architectures, such as node, published topics, subscribed topics, and etc.
There are several `diff` functions, which can be divided into functions that compare the entire architecture and functions that compare nodes within the architecture.

The `diff` function is explained in the following sections. Before using the `diff` functions, the two architecture objects are loaded onto memory beforehand, as shown in the following script.

```python
from caret_analyze import Architecture
left_arch = Architecture('yaml', 'old_architecture.yaml')
right_arch = Architecture('yaml', 'new_architecture.yaml')

```

## How to compare architecture

The functions `diff_node_names()` and `diff_topic_names()` compare two architecture objects by finding the node names and topic names, respectively, that exist only in one of the two objects.
`diff_node_names()` will show you which node is added or deleted in the new version of the target application. `diff_topic_names()` shows you which topic is added or deleted.

### diff_node_names()

The `diff_node_names()` function returns the name of the node whose name appears only in one of the two given architecture objects.

```python
# sample_1

# get node names that are only in left_arch and right_arch respectively
left_only_node_names, right_only_node_names = Architecture.diff_node_names(left_arch, right_arch)

# display differences
print(left_only_node_names)
print(right_only_node_names)

```

Two architectures are given to `diff_node_names()`. Two tuples containing node names are returned if they have any different nodes. If they are have the same nodes at all, `diff_node_names()` returns two empty tuples.

### diff_topic_names()

The `diff_topic_names()` function returns the name of the topic whose name appears only in one of the two given architecture objects.

```python
# sample_2

# get topic names that are only in left_arch and right_arch respectively
left_only_topics, right_only_topics = Architecture.diff_topic_names(left_arch, right_arch)

# display differences
print(left_only_topics)
print(right_only_topics)

```

Two architectures are given to `diff_topic_names()`. Two tuples containing topic names are returned if they have any different topics. If they are have the same topics at all, `diff_topic_names()` returns two empty tuples.

<prettier-ignore-start>
!!!info
      The architecture object has other elements such as Executor and Callback. However, the functions that get the difference of Executors or Callbacks are not yet implemented. Please contact the developers if needed.
<prettier-ignore-end>

## How to compare node in architecture

The `diff_node_pubs()` and `diff_node_subs()` function find the published and subscribed topics, respectively, that exist only in one of the two nodes.
`diff_node_pubs()` will show you which publishing topic is added or deleted in the new version of the target node. `diff_node_subs()` shows you which subscribed topic is added or deleted.

### diff_node_pubs()

The `diff_node_pubs()` function returns the name of the published topic whose name appears only in one of the two nodes.

```python
# sample_3

left_node = left_arch.get_node('node1')
right_node = right_arch.get_node('node1')

# get publish topic names that are only in left_node and right_node respectively
left_only_pub_topics, right_only_pub_topics = Architecture.diff_node_pubs(left_node, right_node)

# display differences
print(left_only_pub_topics)
print(right_only_pub_topics)

```

Two nodes, whose types are `NodeStructValue`, are given to `diff_node_pubs()`. Two tuples containing topic names are returned if they have any different published topics. If they are have the same topics at all, `diff_node_pubs()` returns two empty tuples.


### diff_node_subs()

The `diff_node_subs()` function returns the name of the subscribed topic whose name appears only in one of the two nodes.

```python
# sample_4

left_node = left_arch.get_node('node1')
right_node = right_arch.get_node('node1')

# get subscribe topic names that are only in left_node and right_node respectively
left_only_sub_topics, right_only_sub_topics = Architecture.diff_node_subs(left_node, right_node)

# display differences
print(left_only_sub_topics)
print(right_only_sub_topics)

```

Two nodes, whose types are `NodeStructValue`, are given to `diff_node_subs()`. Two tuples containing topic names are returned if they have any different subscribed topics. If they are have the same topics at all, `diff_node_subs()` returns two empty tuples.
