# Binary Installation

Visualization packages with CARET can also be installed in binary.

<prettier-ignore-start>
!!! Note
    Only the visualization packages can be installed by binary installation.
    The binary does not include measurement packages.
<prettier-ignore-end>

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

The following packages are required for the CARET binary packages. These dependencies are not automatically installed with the binary installation.

```

sudo apt install python3-bt2
python3 -m pip install -U \
 'pandas>=1.4.0' \
 bokeh \
 jupyterlab \
 multimethod

```

## Install CARET packages

Visualization packages for CARET install: caret_analyze, caret_analyze_cpp_impl, ros2caret, caret_msgs.

```

sudo apt install ros-humble-caret-analyze
sudo apt install ros-humble-caret-analyze-cpp-impl
sudo apt install ros-humble-ros2caret
sudo apt install ros-humble-caret-msgs

```

Please open Jupyter Lab.

```

export PATH=$PATH:~/.local/bin
jupyter-lab

```

Recommend trying CARET functionality by following the tutorial.
```
