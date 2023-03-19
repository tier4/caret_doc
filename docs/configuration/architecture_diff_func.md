# Differences of architectures

The diff function can be used to find the difference between two architectures. Specifically, the diff functions can find data that only exist in each, e.g. node name, pub/sub topic.
There are four diff functions, which can be divided into functions that compare the entire architecture and functions that compare nodes within the architecture.
 <!-- `diff_node_names()`, `diff_topic_names` are functions that take two architectures and compare them as a whole. The two functions compare all node names in the architecture for the former and all pub/sub topic names in the architecture for the latter.

`diff_node_pubs()`, `diff_node_subs()` and `diff_node_callbacks()` are functions that compare two nodes within an architecture. These are functions that compare two given nodes and return the publish topic name, subscribe topic name, or callback name that is only in each. -->

## How to compare architecture

In CARET, the structure of the target application is described in an architecture file. The actual architecture file can be represented as an object of Architecture class. This section describes the functions `diff_node_names()` and `diff_topic_names()`, which compare the contents of two architecture objects. In `diff_node_names()` and `diff_topic_names()`, the functions compare two architectures and return the node or topic names that only each has. Therefore, by using these functions, what one ARCHITECTURE has but the other does not (node names or topic names) are clear and can be compared.
One possible use of these functions is to check the changes between the architecture before and after changes.

The diff function is explained below, and before using the diff functions, the two architecture objects are generated beforehand, as shown in the following script.

```python
from caret_analyze import Architecture
left_arch = Architecture('yaml', 'old_architecture.yaml')
right_arch = Architecture('yaml', 'new_architecture.yaml')

```

### diff_node_names()
An example of the use of `diff_node_names()` is shown in `#sample_1`.
<!-- Suppose arch1 has two nodes 'node1' and 'node2'. arch2 has three nodes 'node1', 'node3' and 'node4'. 
The type of the return value is `Tuple[Tuple[str, ...] , Tuple[str, ...]]` -->


```python
#sample_1

#get node names that are only in old and new respectively.
left_only_node_names, right_only_node_names = Architecture.diff_node_names(left_arch, right_arch)

print(left_only_node_names)
print(right_only_node_names)

#result
#('node2',)
#('node3', 'node4')

```

In this setup, the `left_arch` has two nodes: `node1` and `node2`, and the `right_arch` has three nodes: `node1`, `node3` and `node4`. Therefore, the node name that `left_arch` and `right_arch` have in common is `node1`.
Therefore, the node name that only `left_arch` has is `node2`, and the node names that only `right_arch` has are `node3` and `node4`, which are the return value and output at the end of the script respectively.

### diff_topic_names()

An example of the use of `diff_topic_names()` is shown in `#sample_2`.

```python
#sample_2

#get topic names that are only in old and new respectively.
left_only_topics, right_only_topics = Architecture.diff_topic_names(left_arch, right_arch)

print(left_only_topics)
print(right_only_topics)

#result
#('topic2',)
#('topic3', 'topic4')

```

In this setup, the `left_arch` has two topic names: `topic1` and `topic2`, and the `right_arch` has three topic names: `topic1`, `topic3` and `topic4`. The topic name here is the name of the topic used for publish or subscribe.　Therefore, the topic name that `left_arch` and `right_arch` have in common is `topic1`.
Therefore, the topic name that only `left_arch` has is `topic2`, and the topic names that only `right_arch` has are `topic3` and `topic4`, which are the return value and output at the end of the script respectively.

## How to compare node in architecture

<!-- This section describes the functions `diff_node_pubs()` and `diff_node_subs()`, which compare the contents of two nodes in architecture. -->
`diff_node_pubs()` and `diff_node_subs()` are functions that take nodes in an architecture and compare them. The nodes given can be two nodes in the same architecture or in two different architectures. These functions compare the two given nodes and find the pub/sub topic name that exists only in each of them. Thus, what one node has but the other does not (pub/sub topic name) can be identified and compared.

### diff_node_pubs()

An example of the use of `diff_node_pubs()` is shown in `#sample_3`.

```python
#sample_3

left_node = left_arch.nodes[0]
right_node = right_arch.nodes[0]

#get publish topic names that are only in old and new respectively.
left_only_pub_topics, right_only_pub_topics = Architecture.diff_node_pubs(left_node, right_node)

print(left_only_pub_topics)
print(right_only_pub_topics)

#result
#('topic1',)
#('topic2',)

```
In this case, consider a comparison for the first node (with index 0) of `left_arch` and `right_arch`. In this case, the setup assumes that the `left_node` has `topic1` as the publish topic and the `right_node` has `topic2` as the publish topic.
At this time, there is no common publish topic for both.
Therefore, the publish topic that exists only in the `left_node` is `topic1` and the publish topic that exists only in the `right_node` is `topic2`.
Therefore, these are returned as the names of the publish topics that exist only in each node and are output at the end of the script.

### diff_node_subs()

An example of the use of `diff_node_subs()` is shown in `#sample_4`.

```python
#sample_4

left_node = left_arch.nodes[0]
right_node = right_arch.nodes[0]

#get publish topic names that are only in old and new respectively.
left_only_pub_topics, right_only_pub_topics = Architecture.diff_node_subs(left_node, right_node)

print(left_only_sub_topics)
print(right_only_sub_topics)

#result
#('topic1',)
#('topic2',)

```
In this case, consider a comparison for the first node (with index 0) of `left_arch` and `right_arch`. In this case, the setup assumes that the `left_node` has `topic1` as the subscribe topic and the `right_node` has `topic2` as the subscribe topic.
At this time, there is no common subscribe topic for both.
Therefore, the subscribe topic that exists only in the `left_node` is `topic1` and the subscribe topic that exists only in the `right_node` is `topic2`.
Therefore, these are returned as the names of the subscribe topics that exist only in each node and are output at the end of the script.