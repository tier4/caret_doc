# Agnocast Support

[Agnocast](https://github.com/tier4/agnocast) is a rclcpp-compatible true zero-copy IPC middleware. CARET has been extended to support applications using Agnocast as an alternative to DDS, enabling the use of CARET's existing [runtime recording](../runtime_processing/runtime_recording.md), [tracepoint filtering](../runtime_processing/tracepoint_filtering.md), and analysis capabilities with Agnocast-based systems. The usage of CARET remains unchanged when working with Agnocast-based applications.

Please refer to the following PRs for the implementations:

- [tier4/caret_trace#316](https://github.com/tier4/caret_trace/pull/316) - Runtime recording for Agnocast
- [tier4/caret_trace#318](https://github.com/tier4/caret_trace/pull/318) - Trace filtering for Agnocast
- [tier4/caret_analyze#577](https://github.com/tier4/caret_analyze/pull/577) - Agnocast data processing and analysis
- [tier4/caret_trace#326](https://github.com/tier4/caret_trace/pull/326) - Runtime recording and trace filtering for Agnocast node tracepoints 
- [tier4/caret_analyze#587](https://github.com/tier4/caret_analyze/pull/587) - Agnocast node tracepoints data processing and analysis

## Details

For more details on the tracepoint definitions, please refer to the following documents:

- [Agnocast Initialization Tracepoints](./trace_points/agnocast_initialization_tracepoints.md)
- [Agnocast Runtime Tracepoints](./trace_points/agnocast_runtime_tracepoints.md)

## Version Compatibility

CARET supports only Agnocast versions later than [2.1.2](https://github.com/tier4/agnocast/releases/tag/2.1.2).
