# Recording with CARET

## Introduction

Recording includes only two steps; building an target application from source code and executing the target application. After execution, you will find trace data which has recorded events captured by trace points.

If trace data is not recorded due to any reason, you will not be able to analyze the target application as expected. In order to avoid it, CARET serves function to validate trace data.

## Steps to record trace data with CARET

- Build a target application with CARET and check it ([See details](./build_check.md))
- (Optional) Configure trace filter ([See details](./trace_filtering.md))
- Record the application ([See details](./recording.md))
- Validate trace data ([See details](./validating.md))

## Other tips

- [CLI tools](./cli_tool.md)
- [ROS time (sim_time) support](./sim_time.md)
