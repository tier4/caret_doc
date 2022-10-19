# Usage of LTTngEventFilter

CARET can handle measurements and analysis up to several minutes. Memory usage, analysis time, and visualization time, etc., will be issues when leveraging measurement results of longer duration.

Therefore, CARET has an API (LTTngEventFilter) to omit unnecessary measurement results when reading LTTng.  
This section describes how to use the LTTngEventFilter.

LTTngEventFilter has the following filters

- init_pass_filter
- duration_filter
- strip_filter

## API Description

```python
LttngEventFilter.init_pass_filter()
```

- Filter the trace points at initialization

```python
LttngEventFilter.duration_filter(duration_s: float, offset_s: float)
```

- [duration_s] : Measure duration
- [offset_s] : Ignore seconds from start

```python
LttngEventFilter.strip_filter(lsplit_s: Optional[float], rsplit_s: Optional[float])
```

- Ignore measurement results for [lsplit_s] from the start of the measurement
- Ignore measurement results for [rsplit_s] seconds before measurement ends

## Use cases

```python
from caret_analyze import Lttng, LttngEventFilter

lttng = Lttng('/path/to/ctf', event_filters=[
  LttngEventFilter.duration_filter(10, 5)
]) # Filtering from 5 seconds to 10 seconds after the start of the measurement
```

Multiple event_filters can be specified.  
If you include more than one, all filters will be filtered to the specified interval only.
