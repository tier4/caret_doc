# How to define inter-nodes data path

Latency definition section is empty just after an architecture object is loaded from a set of CTF-based recorded data. If you want to observe data flow on targeted paths, you should add the target paths to the architecture object. As a path is combination of multiple nodes and topics, it may be laborious to list their names. To mitigate such burden, CARET serves a function to search the targeted path. The targeted path is added to the architecture object via Python API.

## Basic usage to find and add target path

Listing nodes and topics by their names is laborious. CARET serve a helpful method, `arch.search_paths`,  to search candidates that you want to observe.

The following sample code shows usage of `arch.search_paths` method.

```python
# Architecture object is loaded to variable of arch 

paths = arch.search_paths('source_node',
                          'destination_node')

type(paths) # list of multiple paths
paths[0].summary.pprint() # shows nodes and topics in paths[0]
```

In the sample, `paths` is a list including all possible paths between `source_node` and `destination_node`. If you are satisfied with output from `paths[0].summary.pprint()`, you have to add `paths[0]` to the `arch` object as below.

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

### Efficient search target path

As explained above, `Architecture.search_paths()` returns list of multiple paths.
The list size will be too large to find target path if application has large numbers of nodes and distance between source and destination node is long. In worse case, `Architecture.search_paths()` keep searching and does not return `paths` variable after hours passes.

`Architecture.search_paths` method serves four options to narrow down possible paths as below.

1. Adding another node on the way from source to destination
2. Adding possible maximum length of path with `max_node_depth`
3. Adding node filter




