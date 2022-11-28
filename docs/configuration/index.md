# Configuration

## Introduction

Configuration is a phase where users tell CARET which data path is targeted for visualization.

CARET is capable of recording enormous numbers of events even when a large application runs. The large application may have lots of data paths, but most users must not want to observe all of data paths. If CARET kindly shows analysis results for all of possible data paths, users will be overwhelmed and exhausted to deal with large amount of results.

For delivering efficient analysis, CARET is designed to show analysis results on demand. CARET serves user functions to select target data paths to fulfill their interests.

In addition to defining targeted path, CARET will ask users to define node latency before users analyze their application with visualization.

The rest of this chapter will explain the following two types of sections;

- The detailed background of configuration
- The basic procedure for preparing desired configuration

In detail, the following sections are listed as below.

- [**Background of configuration**](./background.md) section will explain detailed background of configuration phase
- [**How to load and save**](./load_and_save.md) section will tell you how to load and save configuration
- [**How to define inter-node data path**](./inter_node_data_path.md) section will tell you how to use `architecture.search_paths()`
- [**How to define intra-node data path**](./intra_node_data_path.md) will let you know what `message_context` is. This is advanced topic than others
- [**Practical example with CARET_demos**](./practical_example.md) will demonstrates configuration process on [`CARET_demos`](https://github.com/tier4/CARET_demos)

**Visualization of application structure** will be prepared as an appendix which shows another usage of an architecture object. CARET can show structure of a targeted application while it is a performance analysis tool. This appendix will be disclosed in the near future also.
