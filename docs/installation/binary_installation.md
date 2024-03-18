# Binary Installation

Visualization packages with CARET can also be installed in binary.

<prettier-ignore-start>
!!! Note
    Only the [visualization](https://tier4.github.io/caret_doc/main/visualization/) packages can be installed by binary installation.<br>
    The binary does not include [recording](https://tier4.github.io/caret_doc/main/recording/) packages. You can only analyze trace data.
<prettier-ignore-end>

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

Visualization packages for CARET install: caret_analyze, caret_analyze_cpp_impl, ros2caret, caret_msgs.

```bash
sudo apt install -y \
  ros-humble-caret-analyze \
  ros-humble-caret-analyze-cpp-impl \
  ros-humble-ros2caret \
  ros-humble-caret-msgs
```

Check whether CARET (ros2caret) is enabled.

```bash
source /opt/ros/humble/setup.bash
ros2 caret version
```

<prettier-ignore-start>
!!! Note
    As of 2024.3, the version command cannot be specified in the current release of ros2caret.<br>
    A fix for this bug will be released in the near future.<br>
    You can confirm that the installation was successful by using the alternative method.
    ```
    root@f2fd836ae725:/# source /opt/ros/humble/setup.bash
    root@f2fd836ae725:/# ros2 pkg list | grep caret
    caret_analyze
    caret_analyze_cpp_impl
    caret_msgs
    ros2caret
    ```
<prettier-ignore-end>

Recommend trying CARET functionality by following the tutorial.
