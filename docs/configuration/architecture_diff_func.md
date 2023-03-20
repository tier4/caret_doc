# Differences of architectures

The diff function can be used to find the difference between two architectures. Specifically, the diff functions can find data that only exist in each, e.g. node name, pub/sub topic.
There are four diff functions, which can be divided into functions that compare the entire architecture and functions that compare nodes within the architecture.

The diff function is explained below, and before using the diff functions, the two architecture objects are generated beforehand, as shown in the following script.

```python
from caret_analyze import Architecture
left_arch = Architecture('yaml', 'old_architecture.yaml')
right_arch = Architecture('yaml', 'new_architecture.yaml')

```


## How to compare architecture

Architecture objects are generated from architecture files or trace data and summarize the structure of target applications.
The functions `diff_node_names()` and `diff_topic_names()` compare two architecture objects by finding the node names and topic names that exist only in one of the two objects. 
If there are no return values, it means that the two architecture objects represent the same architecture.



### diff_node_names()
The function `diff_node_names()` compares node names of architectures.

```python
# sample_1

# get node names that are only in left_arch and right_arch respectively
left_only_node_names, right_only_node_names = Architecture.diff_node_names(left_arch, right_arch)

# display differences
print(left_only_node_names)
print(right_only_node_names)

```

This function inputs two architectures, compares them, and outputs the node names that exist only in one of architectures.

### diff_topic_names()

The function `diff_topic_names()` compares pub/sub topic names of architectures.

```python
# sample_2

# get topic names that are only in left_arch and right_arch respectively
left_only_topics, right_only_topics = Architecture.diff_topic_names(left_arch, right_arch)

# display differences
print(left_only_topics)
print(right_only_topics)

```

This function inputs two architectures, compares them, and outputs the pub/sub topic names that exist only in one of architectures.

## How to compare node in architecture


`diff_node_pubs()` and `diff_node_subs()` are functions that take nodes in an architecture and compare them. The nodes given can be two nodes in the same architecture or in two different architectures. These functions compare the two given nodes and find the pub/sub topic name that exists only in each of them. Thus, what one node has but the other does not (pub/sub topic name) can be identified and compared.

### diff_node_pubs()

The function `diff_node_pubs()` compares publish topic names of nodes of architectures.

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
This function inputs two nodes of architectures, compares them, and outputs the publish topic names that exist only in one of nodes.

### diff_node_subs()

The function `diff_node_subs()` compares subscribe topic names of nodes of architectures.

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
This function inputs two nodes of architectures, compares them, and outputs the subscribe topic names that exist only in one of nodes.