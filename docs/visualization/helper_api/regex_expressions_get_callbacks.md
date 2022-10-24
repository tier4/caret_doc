# Regex expressions of get_callbacks()

The function `get_callbacks()` gets a list of callbacks that match the argument string and callback names (callback names defined in architecture file).
If there is no match callbacks, this function may notify similar callback names.
The `get_callbacks()` function can recognize UNIX filename patterns such as '\*' or '?'.
If there is no match callbacks even though using regex expressions, a empty list is returned.

**Usage**

```python
from caret_analyze import Architecture, Application, Lttng

callback1 = app.get_callbacks('/timer_driven_node/callback_0')
callback2 = app.get_callbacks('/timer_driven_node/callback_?')
callback3 = app.get_callbacks('/timer_driven_node/*')
```
