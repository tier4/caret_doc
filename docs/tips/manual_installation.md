# Manual installation

## パッケージのインストール

LTTng のインストール
詳細は [LTTng の公式ドキュメント](https://lttng.org/docs/v2.12/#doc-ubuntu-ppa) を参照。

```bash
sudo apt-add-repository ppa:lttng/stable-2.12
sudo apt-get update
sudo apt-get install lttng-tools liblttng-ust-dev
sudo apt-get install python3-babeltrace python3-lttng
```

ROS 2 Humble のインストール、依存パッケージのインストール。
詳細は [ROS2 公式ドキュメント](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html) を参照。

```bash
sudo apt update && sudo apt install -y \
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
  python3-bt2 \
  wget

python3 -m pip install -U \
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
  setuptools \
  colorcet

sudo apt install ros-humble-desktop
```

ros2 trace 関連のパッケージをインストール

```bash
sudo apt install -y \
  ros-humble-ros2trace \
  ros-humble-ros2trace-analysis \
  ros-humble-tracetools \
  ros-humble-tracetools-analysis \
  ros-humble-tracetools-launch \
  ros-humble-tracetools-read \
  ros-humble-tracetools-test \
  ros-humble-tracetools-trace
```

caret の依存パッケージをインストール

```bash
sudo apt update && sudo apt install -y \
  graphviz \
  graphviz-dev

python3 -m pip install -U \
  pytest-mock \
  pybind11 \
  'pandas>=1.4.0' \
  bokeh \
  pandas-bokeh \
  jupyterlab \
  graphviz

# julyterlabのインストール時に[ImportError: The Jupyter Server requires tornado >=6.1.0]と出る場合は以下を実行すること
# pip install tornado --upgrade
```

## CARET のビルド

```bash
mkdir -p ~/ros2_caret_ws/src
cd ~/ros2_caret_ws

wget https://raw.githubusercontent.com/tier4/caret/main/caret.repos
vcs import src < caret.repos

rosdep install \
  --from-paths src --ignore-src \
  --rosdistro humble -y \
  --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# [ERROR: the following packages/stacks could not have their rosdep keys resolved] と出る場合は、以下を試すこと
# rosdep init
# rosdep update
source /opt/ros/humble/setup.bash

# フォークしたパッケージのヘッダーファイルを使用させるためのシンボリックリンクを作成
ln -sf ~/ros2_caret_ws/src/ros-tracing/ros2_tracing/tracetools/include/tracetools ~/ros2_caret_ws/src/ros2/rclcpp/rclcpp/include/
ln -sf ~/ros2_caret_ws/src/ros2/rclcpp/rclcpp_action/include/rclcpp_action ~/ros2_caret_ws/src/ros2/rclcpp/rclcpp/include/
ln -sf ~/ros2_caret_ws/src/ros2/rclcpp/rclcpp_components/include/rclcpp_components/ ~/ros2_caret_ws/src/ros2/rclcpp/rclcpp/include/
ln -sf ~/ros2_caret_ws/src/ros2/rclcpp/rclcpp_lifecycle/include/rclcpp_lifecycle/ ~/ros2_caret_ws/src/ros2/rclcpp/rclcpp/include/

# CARETのビルド
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off  --symlink-install
```

ros2 tracing が有効になっていることを確認

```bash
$ source ~/ros2_caret_ws/install/local_setup.bash
$ ros2 run tracetools status
Tracing enabled
```
