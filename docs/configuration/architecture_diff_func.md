# Differences of architectures

The diff function can be used to find the difference between two architectures. Specifically, the diff functions can find data that only exist in each, e.g. node name, pub/sub topic.
There are four diff functions, which can be divided into functions that compare the entire architecture and functions that compare nodes within the architecture.
 <!-- `diff_node_names()`, `diff_topic_names` are functions that take two architectures and compare them as a whole. The two functions compare all node names in the architecture for the former and all pub/sub topic names in the architecture for the latter.

`diff_node_pubs()`, `diff_node_subs()` and `diff_node_callbacks()` are functions that compare two nodes within an architecture. These are functions that compare two given nodes and return the publish topic name, subscribe topic name, or callback name that is only in each. -->

## How to compare architecture

In CARET, the structure of the target application is described in an architecture file. The actual architecture file can be represented as an object of Architecture class. This section describes the functions `diff_node_names()` and `diff_topic_names()`, which compare the contents of two architecture objects.
One possible use of these functions is to check the changes between the architecture before and after changes.

### diff_node_names()
An example of the use of `diff_node_names()` is shown in `#sample_1`.
Suppose arch1 has two nodes 'node1' and 'node2'. arch2 has three nodes 'node1', 'node3' and 'node4'. 
The type of the return value is `Tuple[Tuple[str, ...] , Tuple[str, ...]]`


```python
#sample_1

from caret_analyze import Architecture

#arch1 has two nodes: 'node1' and 'node2'.
arch1 = Architecture('yaml', 'old_architecture.yaml')

#arch2 has three nodes: 'node1', 'node3', and 'node4'
arch2 = Architecture('yaml', 'new_architecture.yaml')

#get node names that are only in old and new respectively.
old_only_node_names, new_only_node_names = \
            Architecture.diff_node_names(arch1, arch2)

print(old_only_node_names)
print(new_only_node_names)

#result
#('node2',)
#('node3', 'node4')


```

### diff_topic_names()

An example of the use of `diff_topic_names()` is shown in `#sample_2`.
Suppose arch1 has two pub/sub topics: 'topic1' and 'topic2'. arch2 has three pub/sub topics: 'topi1c', 'topic3' and 'topic4'. The pub/sub topic here is the topic used in publish or subscribe.
The type of the return value is `Tuple[Tuple[str, ...] , Tuple[str, ...]]`

```python
#sample_2

from caret_analyze import Architecture

#arch1 has two pub/sub topics: 'topic1' and 'topic2'.
arch1 = Architecture('yaml', 'old_architecture.yaml')

#arch2 has three pub/sub topics: 'topic1', 'topic3', and 'topic4'
arch2 = Architecture('yaml', 'new_architecture.yaml')

#get topic names that are only in old and new respectively.
old_only_topics, new_only_topics = Architecture.diff_topic_names(arch1, arch2)

print(old_only_topics)
print(new_only_topics)

#result
#('topic2',)
#('topic3', 'topic4')

```

## How to compare node in architecture

This section describes the functions `diff_node_pubs()` and `diff_node_subs()`, which compare the contents of two nodes in architecture.

### diff_node_pubs()

An example of the use of `diff_node_pubs()` is shown in `#sample_3`.
Suppose arch1 has two publish topics: 'topic1' and 'topic2'. arch2 has three publish topics: 'topic1', 'topic3' and 'topic4'. 
The type of the return value is `Tuple[Tuple[str, ...] , Tuple[str, ...]]`

```python
#sample_3

from caret_analyze import Architecture

#arch1 has two publish topics: 'topic1' and 'topic2'.
arch1 = Architecture('yaml', 'old_architecture.yaml')

#arch2 has three publish topics: 'topic1', 'topic3', and 'topic4'
arch2 = Architecture('yaml', 'new_architecture.yaml')

#get publish topic names that are only in old and new respectively.
old_only_publish_topics, new_only_publish_topics = \
                Architecture.diff_node_pubs(arch1, arch2)

print(old_only_publish_topics)
print(new_only_publish_topics)

#result
#('topic2',)
#('topic3', 'topic4')

```

### diff_node_subs()

An example of the use of `diff_node_subs()` is shown in `#sample_4`.
Suppose arch1 has two subscribe topics: 'topic1' and 'topic2'. arch2 has three subscribe topics: 'topic1', 'topic3' and 'topic4'. 
The type of the return value is `Tuple[Tuple[str, ...] , Tuple[str, ...]]`

```python
#sample_4

from caret_analyze import Architecture

#arch1 has two subscribe topics: 'topic1' and 'topic2'.
arch1 = Architecture('yaml', 'old_architecture.yaml')

#arch2 has three subscribe topics: 'topic1', 'topic3', and 'topic4'
arch2 = Architecture('yaml', 'new_architecture.yaml')

#get subscribe topic names that are only in old and new respectively.
old_only_subscribe_topics, new_only_subscribe_topics = \
                    Architecture.diff_node_subs(arch1, arch2)

print(old_only_subscribe_topics)
print(new_only_subscribe_topics)

#result
#('topic2',)
#('topic3', 'topic4')

```
