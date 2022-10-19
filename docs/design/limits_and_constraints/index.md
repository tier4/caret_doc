# Limits and constraints

## Environment

CARET only supports environments as follows.

- Support single host.
- FastDDS or CycloneDDS.
- Support for Linux.
- Support for Galactic and Humble.
- Require rebuilding of an application.

## ROS 2 functions

CARET cannot support functions as follows.

- /rosout and /parameter_event topic
- Services
- Actions

## Implementation

CARET cannot support implementations as follows.

- There exist several nodes which have the same namespace and node name.
- wrapper for ROS layers.
- Reentrant callback group.
