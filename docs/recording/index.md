# Recording with CARET

## Introduction

Recording includes two sub-steps at least; building a target application from source code and executing the target application. After execution, you will find trace data which has events data sampled by tracepoints.

Trace data validation is necessary as the third step because any failure on recording prevents users from analyzing performance correctly. Trace data is not recorded correctly due to any reason if a target application is large or if there is an error in the setup procedure. CARET serves functions to validate trace data for the step.

CARET supports trace filter to exclude certain nodes or topics during recording. It is optional, but strongly recommended for analysis of a large application.

## Steps to record trace data with CARET

- Build a target application with CARET and check it ([See details](./build_check.md))
- (Optional) Configure trace filter ([See details](./trace_filtering.md))
- Record the application ([See details](./recording.md))
- Validate trace data ([See details](./validating.md))

## Other tips

- [CLI tools](./cli_tool.md)
- [ROS time (sim_time) support](./sim_time.md)
