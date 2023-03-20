# How to get differences of architectures

The diff function can be used to find the difference between two architectures. Specifically, the diff functions can find data that exist only in one of the architectures, such as node names, publish/subscribe topics, etc. 
There are four diff functions, which can be divided into functions that compare the entire architecture and functions that compare nodes within the architecture.

The diff function is explained below, and before using the diff functions, the two architecture objects are generated beforehand, as shown in the following script.

```python
from caret_analyze import Architecture
left_arch = Architecture('yaml', 'old_architecture.yaml')
right_arch = Architecture('yaml', 'new_architecture.yaml')

```


## How to compare architecture

The functions `diff_node_names()` and `diff_topic_names()` compare two architecture objects by finding the node names and topic names that exist only in one of the two objects. 
One possible use of these functions is to check the changes between the architecture before and after changes.




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

<prettier-ignore-start>
!!!info
      The architecture object has other elements such as Executor and Callback. However, the functions that get the difference of Executors or Callbacks are not yet implemented. Please contact the developers if needed.
<prettier-ignore-end>


## How to compare node in architecture


The `diff_node_pubs()` and `diff_node_subs()` function find the publish and subscription topics that exist only in one of the two nodes. 
This function can be compare also two nodes in different architecture objects.

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