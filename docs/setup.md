# 環境構築

## パッケージのインストール

LTTng のインストール
詳細は [LTTng の公式ドキュメント](https://lttng.org/docs/v2.12/#doc-ubuntu-ppa) を参照。

```bash
$ sudo apt-add-repository ppa:lttng/stable-2.12
$ sudo apt-get update
$ sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
$ sudo apt-get install python3-babeltrace python3-lttng
```
ROS 2 galactic のインストール、依存パッケージのインストール。
詳細は [ROS2 公式ドキュメント](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) を参照。

```bash
$ sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
$ python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools
$ sudo apt install ros-galactic-desktop
```

ros2 trace 関連のパッケージをインストール

```bash
$ sudo apt install -y \
  ros-galactic-ros2trace \
  ros-galactic-ros2trace-analysis \
  ros-galactic-tracetools \
  ros-galactic-tracetools-analysis \
  ros-galactic-tracetools-launch \
  ros-galactic-tracetools-read \
  ros-galactic-tracetools-test \
  ros-galactic-tracetools-trace
```

caret の依存パッケージをインストール

```bash
$ sudo apt update && sudo apt install -y \
  graphviz \
  graphviz-dev
$ python3 -m pip install -U \
  pytest-mock \
  pybind11 \
  pandas \
  bokeh \
  jupyterlab \
  graphviz
```



## CARET のビルド

```bash
$ mkdir -p ~/ros2_caret_ws/src
$ cd ~/ros2_caret_ws
$ wget https://raw.githubusercontent.com/tier4/CARET_doc/main/caret.repos
$ vcs import src < caret.repos --recursive
$ rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
$ source /opt/ros/galactic/setup.bash
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off  --symlink-install
```

ros2 tracing が有効になっていることを確認
```bash
$ source ~/ros2_caret_ws/install/local_setup.bash
$ ros2 run tracetools status
Tracing enabled
```
