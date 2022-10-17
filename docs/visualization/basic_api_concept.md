# Basic APIs Concept

## The Visualization Policy of CARET

CARET provides a number of visualization APIs.
Figure 1 shows the API design principles。

（図 1）横軸が測定粒度で縦軸がシステムレベルの図

The horizontal axis represents the granularity of visualization and the vertical axis represents the granularity of the system being measured.  
In the figure, the more the api is to the left, the more detailed can be visualized and the lower the api in the figure, the finer the system to be visualized.

One of the features of CARET is the ability to evaluate systems while flexibly changing the granularity of visualization and target systems.
If the visualization results show a problem in the system, repeat the visualization toward the lower left of the figure can help identify the problem area by usin this feature.

## Design policy for the Plot class (visualization class)

The visualization tools provided by CARET are implemented in the Plot class.

### The main basic design of the Plot class

The usage of each API of the Plot class is unified as follows:

```python3
from caret_analyze.plot import Plot

... # Processing input data

plot = Plot.create_[metrics]_[graph_type]_plot(data)
plot.show()
plot.to_dataframe()
```

The returned plot has two functions: show() and to_dataframe().

The show() function outputs a figure which able to understand the behavior of the system in chronological order.

to_dataframe() returns a table summarizing the data.
This function is mainly used when analyzing based on specific figures.

Note: For detailed input/output options, see[TimeSeriesPlot](https://tier4.github.io/CARET_analyze/latest/plot/#caret_analyze.plot.TimeSeriesPlot)。
