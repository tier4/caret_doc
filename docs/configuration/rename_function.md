# How to rename element in architecture object

Architecture objects have callback, node, path, executor, and topic objects.
CARET serves APIs to update the names of these objects.
The name of the target element in the architecture object is updated via the Python API.

## Python API

CARET serves Python-based APIs to update the name of the target element in an architecture object.

All of the following code snippets can be executed after load environment variables with `source /path/to/ros2_caret_ws/install/setup.bash`.

### Rename callback_name

You can update callback name from `src` to `dst` with `rename_callback` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_callback('src', 'dst')
```

All `callback_name` in architecture object where it is `src` are updated to `dst`.
This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Rename node_name

You can update node name from `src` to `dst` with `rename_node` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_node('src', 'dst')
```

All `node_name` in architecture object where it is `src` are updated to `dst`.
This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Rename path_name

You can update path name from `src` to `dst` with `rename_path` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_path('src', 'dst')
```

All `path_name` in architecture object where it is `src` are updated to `dst`.
This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Rename executor_name

You can update executor name from `src` to `dst` with `rename_executor` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_executor('src', 'dst')
```

All `executor_name` in architecture object where it is `src` are updated to `dst`.
This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).

### Rename topic_name

You can update topic name from `src` to `dst` with `rename_topic` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_topic('src', 'dst')
```

All `topic_name` in architecture object where it is `src` are updated to `dst`.
This update is also reflected in the architecture file output by [`arch.export`](./load_and_save.md#save).
