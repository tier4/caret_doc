## API to get information about each callback

CARET can visualize execution frequency, period and latency along time for each callback and provide them in the pandas DataFrame format.
Several sets of sample program and output are shown in subsequent sections.
In each example, the following commands are executed in advance.

```python
from caret_analyze import Architecture, Application, Lttng
from caret_analyze.plot import Plot

arch = Architecture('lttng', './e2e_sample')
lttng = Lttng('./e2e_sample')
app = Application(arch, lttng)
```

Please see [CARET analyze API document](https://tier4.github.io/CARET_analyze/latest/plot/) for details on the arguments of this API.
