# How to load and save an architecture object

The first step of configuration is to load an architecture object from a set of CTF-based recorded data onto memory. You can update the architecture object as the following section explains. After you finish updating it, you can save it to a yaml-based file, called "architecture file" to reuse the updated object.

An architecture file has structure of a targeted application. [`dear_ros_node_viewer`](https://github.com/takeshi-iwanari/dear_ros_node_viewer) help you to comprehend structure of the application with the architecture file.

## Python API

CARET serves Python-based APIs to load and save an architecture object.

All of the following code snippets can be executed after load environment variables with `source /path/to/ros2_caret_ws/install/setup.bash`.

### Load from a set of CTF-based recorded data

You can load an architecture object with `Architecture` constructor.

```python
from caret_analyze import Architecture

arch = Architecture('lttng', '/path/to/ctf-based_recorded_data')
```

You will find `caret_analyze.architecture.Architecture`-based object, named '`arch`'.

Loading an architecture object from CTF-based recorded data tends to be time-consuming task.

### Load from a YAML-based architecture file

As I mentioned, CARET serves a function to stored an architecture object into YAML-based architecture file for reusability. It can save loading time and preserve update of the object. You can load it from YAML-based file with only replacing '`lttng`' of the first argument of `Architecture` constructor by '`yaml`'.

```python
from caret_analyze import Architecture

arch = Architecture('yaml', '/path/to/architecture.yaml')
```

It is recommended for you to use YAML-based file to benefit from reusability unless structure of targeted application is changed.

### Save

CARET provides `Architecture.export` method to save an architecture object as follow.

```python
# arch is caret_analyze.architecture.architecture.Architecture-based object


arch.export('/path/to/destination/architecture.yaml')

! readlink -f /path/to/destination//architecture.yaml
# /path/to/destination/architecture.yaml

```

The argument of `arch.export()` is string type and means file path to store the `arch` object. In this sample, `architecture.yaml` will be created in `/path/to/destination` directory if the destination path is writable or another file of the same name does exist.

`arch.export()` has the second argument, `force`, to allow you to overwrite the `arch` object into an existing file. The following sample shows how to overwrite.

```python

arch.export('/path/to/destination/architecture.yaml', force=True)

! readlink -f /path/to/destination//architecture.yaml
# /path/to/destination/architecture.yaml
```

`force=True` option erases the existing architecture object.

## CLI

### Create an architecture file via CLI

With the functions I introduced above, you can create a YAML-based file including an architecture object. CARET serves CLI to create it as well. `create_architecture_file` command plays role of it.

The following sample code shows how to use `create_architecture_file` command.

```bash
source /path/to/ros2_caret_ws/install/setup.bash

ros2 caret create_architecture_file /path/to/ctf-based_recorded_data -o /path/to/destination/architecture.yaml

readlink -f /path/to/destination/architecture.yaml
# /path/to/destination/architecture.yaml
```
