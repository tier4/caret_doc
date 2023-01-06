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
2. **Limiting maximum number of nodes** between given nodes with `max_node_depth`
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

#### Limiting maximum number of nodes

`Architecture.search_paths()` will scan all paths as possible if you don't give `max_node_depth` argument. `max_node_depth` means maximum number between given nodes. The number of candidate paths will be suppressed by this argument.

This argument will be helpful When you waste much time for `Architecture.search_paths()`.

The usage is shown as below. This can be used with another approach to filter candidates.

```python
# Architecture object is loaded to variable of arch

paths = arch.search_paths('source_node',
                          'intermediate_node_1',
                          'destination_node',
                          max_node_depth=10)

```

`max_node_depth` does not always limit the maximum number of nodes between source and destination. If you give 3 nodes to `arch.search_paths` as shown above, `max_node_depth` does not limits the maximum number of nodes between `source_node` and `destination_node`. In this example, `max_node_depth` limits the number of nodes between `source_node` and `intermediate_node_1`, and that between `intermediate_node_1` and `destination_node`.

#### Node and topic filter

As node filter is similar to communication filter, they are explained together in this section.

With node filter and communication filter, `Architecture.search_paths()` excludes paths which includes selected nodes or topics. They support regular expression.

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
    '/intermediate_node'
    '/end_node',
    max_node_depth=30,
    node_filter = node_filter,
    communication_filter = comm_filter)
```

### Path combining

If the path from `source node` to `destination node` is too long, `Architecture.search_paths()` takes a long time and may find many paths that don't need to be considered.
It may be laborious to select a correct target path.
`Architecture.combine_path()` combines two paths which are found by `Architecture.search_paths()`.
By searching short paths and combining them repeatedly, you can get a target path. It is more efficient than searching a longer path directly according to "divide-and-conquer" method.

Usage of `Architecture.combine_path()` is as following.

```python
paths_1 = arch.search_paths('source_node',
                            'intermediate_node')
paths_2 = arch.search_paths('intermediate_node',
                            'destination_node')
target_path = arch.combine_path(paths_1[0], paths_2[0])

arch.add_path('combined_path_name', target_path)
arch.export('new_architecture.yaml')
```

An exception is raised for pairs that cannot be combined.
