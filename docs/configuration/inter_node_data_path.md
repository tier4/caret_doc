# How to define inter-nodes data path

Latency definition section is empty just after an architecture object is loaded from a set of CTF-based recorded data. If you want to observe data flow on targeted paths, you should add the target paths to the architecture object. As a path is combination of multiple nodes and topics, it may be laborious to list their names. To mitigate such burden, CARET serves a function to search the targeted path. The targeted path is added to the architecture object via Python API.

## Basic usage to find and add target path

Listing nodes and topics by their names is laborious. CARET serve a helpful method, `arch.search_paths`, to search candidates that you want to observe.

The following sample code shows usage of `arch.search_paths` method.

```python
# Architecture object is loaded to variable of arch

paths = arch.search_paths('source_node',
                          'destination_node')

type(paths) # list of multiple paths
paths[0].summary.pprint() # shows nodes and topics in paths[0]
```

In the sample, `paths` is a list including all possible paths between `source_node` and `destination_node`, whose type is `PathStructValue`. If you are satisfied with output from `paths[0].summary.pprint()`, you have to add `paths[0]` to the `arch` object as below.

```python
arch.add_path('target_path', paths[0])

arch.export('new_architecture_with_path.yaml')
```

In the above sample, `paths[0]` is named as `target_path` and registered to the `arch` object. `arch` object is exported to a new architecture file for reuse.

If you want to restore the `paths[0]` object with `new_architecture_with_path.yaml` file, `arch.get_path()` method will help you to do so.

```python
arch = Architecture('yaml', 'new_architecture_with_path.yaml')

path = arch.get_path('target_path') # path object is same as paths[0] in the previous sample
```

### Efficient target path search

As explained above, `Architecture.search_paths()` returns list of multiple paths.
The list size will be too large to find target path if application has large numbers of nodes and distance between source and destination node is long. In worse case, `Architecture.search_paths()` keep searching and does not return `paths` variable after hours passes.

`Architecture.search_paths()` method serves four options to narrow down possible paths as below.

1. **Additional nodes** as variable length arguments
2. **`max_node_depth`** for limiting maximum number of nodes in path
3. **Node filter** which excludes paths including specific nodes
4. **Communication filter** which excludes paths including specific topics

In short, `Architecture.search_paths` is defined as follows.

```python
search_paths(
    *node_names: 'str',
    max_node_depth: 'Optional[int]' = None,
    node_filter: 'Optional[Callable[[str], bool]]' = None,
    communication_filter: 'Optional[Callable[[str], bool]]' = None
) -> 'List[PathStructValue]'
```

The following sub-sections will explain their roles and usages in details.

#### Additional nodes

In the previous example, `Architecture.search_paths()` had two arguments `source_node` and `destination_node`. However, the number of nodes given to `Architecture.search_paths()` is variable and not always two. You can add other nodes to `Architecture.search_paths()` as below, and you will get a list including multiple paths which passes all given nodes.

```python
# Architecture object is loaded to variable of arch

paths = arch.search_paths('source_node',
                          'intermediate_node_1',
                          'intermediate_node_2',
                          'destination_node')
```

`paths` is a list including multiple paths which pass `source_node`, `intermediate_node_1`, `intermediate_node_2`, and `destination_node`. They are allowed to pass another node, but all chosen nodes are passed in order.

#### `max_node_depth`

`Architecture.search_paths()` will scan all paths as possible if you don't give `max_node_depth` argument. `max_node_depth` means maximum number of nodes including a path. The number of candidate paths will be suppressed by this argument.

This argument will be helpful When you waste much time for `Architecture.search_paths()`.

The usage is shown as below. This can be used with another approach to filter candidates.

```python
# Architecture object is loaded to variable of arch

paths = arch.search_paths('source_node',
                          'intermediate_node_',
                          'destination_node',
                          max_node_depth=10)

```

#### Node and topic filter

As node filter is similar to communication filter, they are explained together in this section.

With node filter and communication filter, `Architecture.search_paths()` excludes paths which includes selected nodes and topics. They support regular expression.

The following sample code shows usage.

```python
import re

# name list of nodes to be excluded
node_filters = [
    re.compile(r'/_ros2cli_/*'),
    re.compile(r'/launch_ros_*'),
]

# name list of topics to be excluded
comm_filters = [
    re.compile(r'/tf/*'),
]
def comm_filter(topic_name: str) -> bool:
    can_pass = True
    for comm_filter in comm_filters:
        can_pass &= not bool(comm_filter.search(topic_name))
    return can_pass

def node_filter(node_name: str) -> bool:
    can_pass = True
    for node_filter in node_filters:
        can_pass &= not bool(node_filter.search(node_name))
    return can_pass

paths = arch.search_paths(
    '/start_node',
    '/end_node',
    max_node_depth=30,
    node_filter = node_filter,
    communication_filter = comm_filter)
```
