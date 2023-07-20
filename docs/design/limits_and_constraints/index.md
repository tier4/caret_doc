# Limits and constraints

## Environment

CARET only supports environments as follows.

- Support single host
- Support FastDDS or CycloneDDS
- Support only Linux, especially Ubuntu
- Support Galactic and Humble
- Support only applications build with rclcpp library
- Support only processes acting as node
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
