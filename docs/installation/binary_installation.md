# Binary Installation

Visualization packages with CARET can also be installed in binary.
CARET binary packages are currently only installable on Ubuntu Jammy and ROS 2 Humble.

<prettier-ignore-start>
!!! Note
    The binary does not include measurement packages.
<prettier-ignore-end>

## Installation of required packages

Update your apt repository caches.

```
sudo apt update
```

Install packages for visualization with CARET.

```
sudo apt install python3-bt2
python3 -m pip install -U \
  'pandas>=1.4.0' \
  bokeh \
  jupyterlab \
  multimethod
```

## Install CARET packages

Analysis packages for CARET install: caret_analyze, caret_analyze_cpp_impl, ros2caret, caret_msgs.

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
