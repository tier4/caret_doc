# Using get_callbacks function

The get_callbacks() is a function that gets callback information.
Callbacks information includes callback name, callback period[ns], callback type.


## API

```python
get_callbacks(self, *callback_names: str)
```

- The 'get_callbacks()' function gets callbacks that match the argument string and callback name.
- If there is no match callbacks, The 'get_callbacks()' function warn and may show similar callbacks name when there is no match callbacks.
- The 'get_callbacks()' function can contain '*' or '?' as regular expression. NOTE: The 'get_callbacks()' function can't use other regular expresion like '+'.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callbacks1 = get_callbacks('timer_driven_node/callbacks_0')
callbacks2 = get_callbacks('timer_driven_node/callbacks_?')
callbacks3 = get_callbacks('timer_driven_node/*')
```
