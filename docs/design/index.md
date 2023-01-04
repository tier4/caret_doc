# Design

## Introduction

Design chapter describes details of CARET like architecture overview, definition of tracepoints, integration of data for visualization.
The following figure shows the overview of tracing flow and the related packages.

![tracing flow and packages](../imgs/design.drawio.png)

CARET records events data, including metadata and timestamps, from tracepoints embedded in user applications, ROS 2 and DDS.
The events data are dumped to a set of [CTF](https://diamon.org/ctf/)-based files. The set of files is called "trace data" in the context of CARET.

CARET load trace data and convert them to graphs and statistics for human-beings to comprehend performance and bottleneck of the application.

This chapter describes the design policy and key points as listed below. As the chapter is written for heavy users or developers, most of light users might feel bored.

- Design overview on the architecture and the trace points

  1. [Software architecture](./software_architecture/index.md)
  2. [Event and latency definition](./event_and_latency_definitions/index.md)
  3. [Limits and constraints](./limits_and_constraints/index.md)

- Design related to [Recording](../recording/index.md)

  1. [Runtime processing](./runtime_processing/index.md)
  2. [Tracepoints](./trace_points/index.md)

- Design related to [Configuration]

  1. [Configuration](./configuration/index.md)

- Design related to [Visualization]
  1. [Processing trace data](./processing_trace_data/index.md)
  2. [Visualization](./visualizations/index.md)

If you are interested in details of implementation, [CARET_analyze API document](https://tier4.github.io/CARET_analyze/) might be helpful.
