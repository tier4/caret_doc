## 環境構築
### LTTngのインストール

https://lttng.org/docs/v2.12/#doc-ubuntu-ppa
```
apt-add-repository ppa:lttng/stable-2.12
apt-get update
```
```
sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
sudo apt-get install python3-babeltrace python3-lttng
```

### ビルドに必要なパッケージ　のインストール
```
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
  pytest-mock \
  pybind11 \
  pandas \
  fire
```

### ROS 2 galactic のインストール

```
sudo apt install ros-galactic-desktop
```
詳細は公式ドキュメント参照
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

### ros2 trace 関連のパッケージをインストール
```
sudo apt install -y \
  ros-galactic-ros2trace \
  ros-galactic-ros2trace-analysis \
  ros-galactic-tracetools \
  ros-galactic-tracetools-analysis \
  ros-galactic-tracetools-launch \
  ros-galactic-tracetools-read \
  ros-galactic-tracetools-test \
  ros-galactic-tracetools-trace
```

### CARET のビルド

```
mkdir -p ~/ros2_caret_ws/src
cd ~/ros2_caret_ws
wget https://raw.githubusercontent.com/tier4/CARET_doc/main/caret.repos
vcs import src < caret.repos --recursive
rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
colcon build --symlink-install --packages-select cyclonedds --cmake-args -DCMAKE_BUILD_TYPE=Debug
colcon build --symlink-install --packages-select tracetools rcl rclcpp caret_trace caret_analyze  caret_analyze_cpp_impl
```

ros2 tracing のトレースが有効になっていることを確認
```
. ~/ros2_caret_ws/install/local_setup.bash
$ ros2 run tracetools status
Tracing enabled

```

```
$ mkdir -p ~/ros2_ws/src
$ cd ros2_ws
$ git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive

$ source ~/ros2_caret_ws/install_setup.bash
$ colcon build --symlink-install --packages-up-to caret_demos
```

### 動作確認

```
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/

$ source ./install/setup.bash
$ colcon build --symlink-install

$ export ROS_TRACE_DIR=$(pwd) # トレースファイルの出力をデフォルトから変更
$ ros2 launch caret_demos talker_listener.launch.py
数秒後、 Ctrl+C で終了

$ # ros2_caret が表示されていることを確認
$ babeltrace talker_listener/ | cut -d' ' -f 4 | sort -u
ros2:callback_end:
ros2:callback_start:
ros2_caret:dds_bind_addr_to_stamp:
ros2_caret:dds_write:
ros2_caret:on_data_available:
ros2:dispatch_subscription_callback:
ros2:rclcpp_callback_register:
ros2:rclcpp_publish:
ros2:rclcpp_service_callback_added:
ros2:rclcpp_subscription_callback_added:
ros2:rclcpp_subscription_init:
ros2:rclcpp_timer_callback_added:
ros2:rclcpp_timer_link_node:
ros2:rcl_init:
ros2:rcl_node_init:
ros2:rcl_publish:
ros2:rcl_publisher_init:
ros2:rcl_service_init:
ros2:rcl_subscription_init:
ros2:rcl_timer_init:
```
