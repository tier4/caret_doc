# How to rename sub-objects in an architecture object

An architecture object has several sub-objects such as callbacks, nodes, paths, executors and topics.  
CARET assigns names to these sub-objects to identify them. These names come from identifiers captured at initialization tracepoints. The target application may assign different identifiers to the sub-objects on each launch or update. It prevents users from reusing the existing architecture object as it is. To enhance the reusability of the architecture object, CARET serves APIs to change the names of these sub-objects.

All of the following code snippets can be executed after the `Architecture('type', 'file_path')` method loads an architecture object.
The architecture object whose sub-objects are renamed is saved to a file as explained in [the previous page](./load_and_save.md#save).

## Rename `callback_name`

You can update callback names from `src` to `dst` with `rename_callback` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_callback('src', 'dst')
```

All `callback_name` in architecture object where it is `src` are updated to `dst`.

## Rename `node_name`

You can update node names from `src` to `dst` with `rename_node` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_node('src', 'dst')
```

All `node_name` in architecture object where it is `src` are updated to `dst`.

## Rename `path_name`

You can update path names from `src` to `dst` with `rename_path` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_path('src', 'dst')
```

All `path_name` in architecture object where it is `src` are updated to `dst`.

## Rename `executor_name`

You can update executor names from `src` to `dst` with `rename_executor` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_executor('src', 'dst')
```

All `executor_name` in architecture object where it is `src` are updated to `dst`.

## Rename `topic_name`

You can update topic names from `src` to `dst` with `rename_topic` function in `Architecture` class.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object

arch.rename_topic('src', 'dst')
```

All `topic_name` in architecture object where it is `src` are updated to `dst`.
