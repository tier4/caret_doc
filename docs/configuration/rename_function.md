# How to load and save an architecture object

An architecture object loaded from a set of CTF-based record data or an architecture file has the elements callback, node, path, executor and topic.
If you want to rename these elements, CARET serves the functions to update the name of the target element.
The name of the target element in the architecture object is updated via the Python API.

## Python API

CARET serves Python-based APIs to update the name of the target element in an architecture object.

All of the following code snippets can be executed after load environment variables with `source /path/to/ros2_caret_ws/install/setup.bash`.

### Update callback name in architecture object.

You can update callback name in architecture object from `src` to `dst` with `rename_callback` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object


arch.rename_callback('src', 'dst')
```

All `callback_name` in architecture object where it is `src` are updated to `dst`.

This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Update node name in architecture object.

You can update node name in architecture object from `src` to `dst` with `rename_node` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object


arch.rename_node('src', 'dst')
```

All `node_name` in architecture object where it is `src` are updated to `dst`.

This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Update path name in architecture object.

You can update path name in architecture object from `src` to `dst` with `rename_path` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object


arch.rename_path('src', 'dst')
```

All `path_name` in architecture object where it is `src` are updated to `dst`.

This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Update executor name in architecture object.

You can update executor name in architecture object from `src` to `dst` with `rename_executor` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object


arch.rename_executor('src', 'dst')
```

All `executor_name` in architecture object where it is `src` are updated to `dst`.

This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Update topic name in architecture object.

You can update topic name in architecture object from `src` to `dst` with `rename_topic` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object


arch.rename_topic('src', 'dst')
```

All `topic_name` in architecture object where it is `src` are updated to `dst`.

This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).