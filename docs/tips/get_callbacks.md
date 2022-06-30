# Detailed Behavior Investigation Methods

CARET provides a ROS-friendly class structure that allows the following classes to obtain detailed execution information.

- Executor
- Node
- CallbackGroup
- Callback
- Publisher
- Subscription
- Timer

Please refer to the API documentation (link) for information on what information can be obtained from each class.

This section describes how to obtain the targeted class.

## Description of get_callback

The 'get_callback()' funtion that gets callback information
Callbacks information includes callback name, callback period[ns], callback type.

## API

```python
get_callback(self, callback_names: str)
```

- The 'get_callbacks()' function gets callbacks that match the argument string and callback name.
    - callback name defined in architecture file.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callbacks1 = app.get_callback('timer_driven_node/callbacks_0')
callbacks2 = app.get_callback('timer_driven_node/callbacks_1')
```


## Description of get_callbacks

The get_callbacks() is a function that gets callback information.
Callbacks information includes callback name, callback period[ns], callback type.


## API

```python
get_callbacks(self, *callback_names: str)
```

- The 'get_callbacks()' function gets callbacks that match the argument string and callback name.
    - callback name defined in architecture file.
- If there is no match callbacks, The 'get_callbacks()' function warn and may show similar callbacks name when there is no match callbacks.
- The 'get_callbacks()' function can contain '*' or '?' as regular expression. NOTE: The 'get_callbacks()' function can't use other Unix filename pattern matching like '+'.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callbacks1 = app.get_callbacks('timer_driven_node/callbacks_0')
callbacks2 = app.get_callbacks('timer_driven_node/callbacks_?')
callbacks3 = app.get_callbacks('timer_driven_node/*')
```
