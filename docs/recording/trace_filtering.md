# Trace filtering

## Trace filtering

CARET provides a trace filtering function to exclude specific nodes and topics. Trace data lost may occur when recording a large application like Autoware due to too many data to be recorded, and a warning will happen when validating/analyzing the recorded trace data (See [valiating](../validating/#tracer-discarded)). By applying trace filter, unconcerned events like `/tf` are ignored and the size of recorded data decreases.

## Trace filter configuration

- Trace filter configurtion is performed by setting the following environment variables
  - `CARET_SELECT_NODES` : node names to be recorded
  - `CARET_IGNORE_NODES` : node names to be ignored
  - `CARET_SELECT_TOPICS` : topic names to be recorded
  - `CARET_IGNORE_TOPICS` : topic names to be ignored
- "SELECT" settings override "IGNORE" settings if both are used
- Colon "`:`" is used to set more than one nodes/topics
- Regular expressions are supported
- These variables need to be set in the same terminal as a target application running
- In most cases, nodes related to `/rviz`, `/clock` topic and `/parameter_events` topic are unnecessary to analyze an application. It's recommended to ignore these nodes/topics

The following shows sample settings

```sh
export CARET_IGNORE_NODES="/rviz*"
export CARET_IGNORE_TOPICS="/clock:/parameter_events"
```

<prettier-ignore-start>
!!!note
      Trace filtering doesn't exclude all events of the specified nodes/topics with current implementation. Events in certain layers, especially DDS layer, won't be ignored.
<prettier-ignore-end>

## Trace filter setting file

It will be handy to prepare trace filter setting file like the following.

```sh
# caret_topic_filter.bash
#!/bin/bash

export CARET_IGNORE_NODES=\
"\
/rviz*\
"

export CARET_IGNORE_TOPICS=\
"\
/clock:\
/parameter_events\
"

# if you want to select nodes or topics,
# please remove comment out of the followings.
# export CARET_SELECT_NODES=\
# "\
# /rviz*\
# "

# export CARET_SELECT_TOPICS=\
# "\
# /clock:\
# /parameter_events\
# "
```
