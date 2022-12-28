# Wildcards for get_callbacks()

`get_callbacks()` function gets a list of callbacks whose names match a given string as an argument.
If there is no matched callbacks, this function may tell you possible candidates of callback.
`get_callbacks()` supports UNIX-based wildcard such as '\*' or '?'.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

callback1 = app.get_callbacks('/timer_driven_node/callback_0')
callback2 = app.get_callbacks('/timer_driven_node/callback_?')
callback3 = app.get_callbacks('/timer_driven_node/*')
```
