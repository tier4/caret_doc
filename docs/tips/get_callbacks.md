# Detailed Behavior Investigation Methods

CARET provides a ROS-friendly class structure that allows the following classes to obtain detailed execution information.

- Executor
- Node
- CallbackGroup
- Callback
- Publisher
- Subscription
- Timer

Please refer to the API documentation (https://tier4.github.io/CARET_analyze/latest/) for information on what information can be obtained the target class.

This section describes how to obtain the targeted class.

## Description of get_callback

The 'get_callback()' is a funtion that returns callback information such as callback name, callback period[ns], callback type.

## API

```python
get_callback(self, callback_name: str) -> CallbackBase
```

- The 'get_callback()' function returns single callback that match the argument string and callback name.
If no matching callbacks are found, get_callback raises a exception.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callback1 = app.get_callback('timer_driven_node/callback_0')
callback2 = app.get_callback('timer_driven_node/callback_1')
```


## Description of get_callbacks

The get_callbacks() is a function that gets callback information.
Callbacks information includes callback name, callback period[ns], callback type.


## API

```python
get_callbacks(self, *callback_names: str) -> List[CallbackBase]
```

- The 'get_callbacks()' function gets callbacks that match the argument string and callback name.
    - callback name defined in architecture file.
- If concrete callback names are given and there is no match callbacks, The 'get_callbacks()' function warns and may notify similar callbacks name when there is no match callbacks.
- The 'get_callbacks()' function can recognize UNIX filename pattern such as '*' or '?'.
- If patterns are given, get_callbacks doesn't raise any exception, and returns empty list.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callback1 = app.get_callbacks('timer_driven_node/callback_0')
callback2 = app.get_callbacks('timer_driven_node/callback_?')
callback3 = app.get_callbacks('timer_driven_node/*')
```
