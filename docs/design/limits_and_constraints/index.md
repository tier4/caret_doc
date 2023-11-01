# Limits and constraints

## Environment

CARET only supports environments as follows.

- Support single host
- Support FastDDS or CycloneDDS
- Support only Linux, especially Ubuntu
- Support Galacticm, Humble and Iron
- Require rebuilding of an application with CARET libraries

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
- Other than processing that operates on a node
- Other than nodes built with rclcpp library
