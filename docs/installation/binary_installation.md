# Binary Installation

The packages for analyzing trace data with CARET can also be installed in binary.

<prettier-ignore-start>
!!! Note
    Only the packages for analyzing trace data can be installed by binary installation.<br>
    The binary does not include the packages for recording trace data.
<prettier-ignore-end>

The table of packages installable via binary installation is following.
| packages name                                                                 | packages description                                                          | binary install |
| ----------------------------------------------------------------------------- | ----------------------------------------------------------------------------- | -------------- |
| [caret](https://github.com/tier4/caret)                                       | Meta-repository                                                               | ×              |
| [caret_trace](https://github.com/tier4/caret_trace/tree/main/CARET_trace)     | Define tracepoints added by function hooking                                  | ×              |
| [caret_msgs](https://github.com/tier4/caret_trace/tree/main/caret_msgs)       | Message type defined by caret                                                 | ○              |
| [caret_analyze](https://github.com/tier4/caret_analyze)                       | Library for scripts to analyze and visualize data                             | ○              |
| [caret_analyze_cpp_impl](https://github.com/tier4/caret_analyze_cpp_impl.git) | Efficient helper functions to analyze trace data written in C++               | ○              |
| [ros2caret](https://github.com/tier4/ros2caret.git)                           | CLI commands like `ros2 caret`                                                | ○              |
| [caret_doc](https://github.com/tier4/caret_doc)                               | Documentation                                                                 | ×              |
| [caret_demos](https://github.com/tier4/caret_demos)                           | Demo programs for CARET                                                       | ×              |
| [rclcpp](https://github.com/tier4/rclcpp/tree/v0.3.0)                         | The forked rclcpp including CARET-dedicated tracepoints                       | ×              |
| [ros2_tracing](https://github.com/tier4/ros2_tracing/tree/v0.3.0)             | The forked `ros2_tracing` including definition of CARET-dedicated tracepoints | ×              |

## Requirements

CARET is confirmed to run on the platforms shown in the following table with supported version.

| dependent platform | supported version |
| ------------------ | ----------------- |
| ROS                | Humble            |
| Ubuntu             | 22.04             |
| LTTng              | stable-2.13       |
| Linux Kernel       | 5.15.x            |
| Python3            | 3.10.x            |

## Installation of required packages

Update your apt repository caches.

```bash
sudo apt update
```

The following packages are required for the CARET binary packages. These dependencies are not automatically installed with the binary installation.

```bash
sudo apt install python3-bt2
python3 -m pip install -U \
  pandas>=2.1.1 \
  bokeh>=3 \
  jupyterlab \
  multimethod
```

## Install CARET packages

The packages for analyzing trace data with CARET install.

```bash
sudo apt install -y \
  ros-humble-caret-analyze \
  ros-humble-caret-analyze-cpp-impl \
  ros-humble-ros2caret \
  ros-humble-caret-msgs
```

You can confirm that the installation was successful by using the alternative method.

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep caret
# caret_analyze
# caret_analyze_cpp_impl
# caret_msgs
# ros2caret
```

Recommend trying CARET functionality by following the tutorial.
Prepare trace data and try visualization, following [this tutorial](https://tier4.github.io/caret_doc/main/tutorials/visualization/).
