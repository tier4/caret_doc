# Using get_callbacks function

CARET has a function that get callbacks information.
Callbacks information includes callback name, callback period[ns], callback type.


## API

```python
get_callbacks(self, *callback_names: str)
```

- The 'get_callbacks()' gets callbacks that match the string.
- The 'get_callbacks()' can contain '*' or '?' as regular expression. NOTE: User can't use other regular expresion like '+'.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callbacks1 = get_callbacks('timer_driven_node/callbacks_0')
callbacks2 = get_callbacks('timer_driven_node/callbacks_?')
callbacks3 = get_callbacks('timer_driven_node/*')
```

This function may print similar callbacks name when there is no match callbacks.
