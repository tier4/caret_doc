# Usage of LTTngEventFilter

CARET is able to analyze data recorded for several minutes. When analyzing large data with CARET, both memory and time consumed for analysis will be critical issue.

In order not to load unnecessary data, CARET has `LTTngEventFilter` to exclude selected data from loading data.


`LTTngEventFilter` has the following methods.

- `init_pass_filter`
- `duration_filter`
- `strip_filter`

## `init_pass_filter`

With `init_pass_filter` method, events recorded by initialization tracepoints are not loaded onto memory.

```python
LttngEventFilter.init_pass_filter()
```

## `duration_filter`

With `duration_filter` method, you can crop data with targeting time range of recorded data.

```python
LttngEventFilter.duration_filter(duration_s: float, offset_s: float)
```

`duration_filter` method has following two arguments.

- `duration_s` is duration of target time range
- `offset_s` is  point of target time range

## `strip_filter`

With `strip_filter` method, you can crop data with targeting time range of recorded data as well as `duration_filter`. `strip_filter` has different arguments from `duration_filter`.

```python
LttngEventFilter.strip_filter(lsplit_s: Optional[float], rsplit_s: Optional[float])
```

`strip_filter` method has following two optional arguments. If you omit either or both of them, default value '0' is given to the optional argument.

- `lstrip_s` is start time of cropping range
- `rstrip_s` is end point of cropping range

## Sample code

A sample code, where `duration_filter` is used, is given as below.

```python
from caret_analyze import Lttng, LttngEventFilter

lttng = Lttng('/path/to/ctf', event_filters=[
  LttngEventFilter.duration_filter(10, 5)
]) # Filtering for 10 from 5 seconds
```

If you want to add another filter, append it to `event_filters` list.
