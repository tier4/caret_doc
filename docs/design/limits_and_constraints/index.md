# Limits and constraints

## Environment

CARET only supports environments as follows.

- Support single host
- Support FastDDS or CycloneDDS
- Support only Linux, especially Ubuntu
- Support Galactic and Humble
- Require rebuilding of an application

## ROS 2 functions

CARET does not support the following functions.

- /rosout and /parameter_event topic
- Services
- Actions

## Implementation

CARET does not support implementations as follows.

- Multiple nodes whose full-names are same
- Wrapper for ROS layers
- Reentrant callback group
