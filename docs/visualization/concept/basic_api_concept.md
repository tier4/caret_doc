# Basic APIs Concept

## The main basic design of the Plot class

The visualization tools provided by CARET are implemented in the Plot class.
The usage of each API of the Plot class is unified as follows:

```python3
from caret_analyze.plot import Plot

... # Processing input data

plot = Plot.create_[metrics]_[graph_type]_plot(data)
plot.show()
plot.to_dataframe()
```

The variable "plot" has two functions, show() and to_dataframe().
The function "show()" outputs a figure and returns the figure's handler.
The function "to_dataframe()" returns a table summarizing the data.
This function is mainly used to analyze based on specific figures.

Note: For detailed input/output options, see [TimeSeriesPlot](https://tier4.github.io/CARET_analyze/latest/plot/#caret_analyze.plot.TimeSeriesPlot).
