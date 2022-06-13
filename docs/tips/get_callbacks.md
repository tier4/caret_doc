# Using get_callbacks function

CARET has a function that get callbacks.
We describe get_callbacks().


## API

```python
get_callbacks(self, *callback_names: str)
```

- It gets callbacks that match the string.
- It can contain '*' or '?' as regular expression. NOTE: You can't use other regular expresion like '+'.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callbacks1 = get_callbacks('timer_driven_node/callbacks_0')
callbacks2 = get_callbacks('timer_driven_node/callbacks_?')
callbacks3 = get_callbacks('timer_driven_node/*')
```

This function may print similar callbacks name when there is no match callbacks.
