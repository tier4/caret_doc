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

### 依存パッケージのインストール
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
  wget \
  graphviz \
  graphviz-dev

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
  fire \
  bokeh \
  pygraphviz
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
colcon build --symlink-install --packages-skip cyclonedds
```

ros2 tracing のトレースが有効になっていることを確認
```
. ~/ros2_caret_ws/install/local_setup.bash
$ ros2 run tracetools status
Tracing enabled

```


### 動作確認

デモアプリのビルド
```
$ mkdir -p ~/ros2_ws/src
$ cd ros2_ws
$ git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive

$ source ~/ros2_caret_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-up-to caret_demos
```
測定
```
$ source ~/ros2_ws/install/local_setup.bash
$ export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
$ export ROS_TRACE_DIR=$(pwd) # トレースファイルの出力をデフォルトから変更
$ ros2 launch caret_demos talker_listener.launch.py # 数秒後、 Ctrl+C で終了
[INFO] [launch]: All log files can be found below /home/hasegawa/.ros/log/2021-09-09-20-07-26-339549-hasegawa-System-Product-Name-117168
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [tracetools_launch.action]: Writing tracing session to: /home/hasegawa/ros2_ws/talker_listener
[INFO] [talker-1]: process started with pid [117177]
[INFO] [listener-2]: process started with pid [117179]
[listener-2] 1631185646.549326 [0]   listener: using network interface enp0s31f6 (udp/10.22.163.187) selected arbitrarily from: enp0s31f6, docker0
[talker-1] 1631185646.553565 [0]     talker: using network interface enp0s31f6 (udp/10.22.163.187) selected arbitrarily from: enp0s31f6, docker0
[talker-1] [INFO] [1631185647.560143733] [talker]: Publishing: 'Hello World: 1'
[listener-2] [INFO] [1631185647.561448008] [listener]: I heard: [Hello World: 1]
[talker-1] [INFO] [1631185648.559836958] [talker]: Publishing: 'Hello World: 2'
[listener-2] [INFO] [1631185648.560450087] [listener]: I heard: [Hello World: 2]
[talker-1] [INFO] [1631185649.559899105] [talker]: Publishing: 'Hello World: 3'
[listener-2] [INFO] [1631185649.560591699] [listener]: I heard: [Hello World: 3]
[talker-1] [INFO] [1631185650.559793529] [talker]: Publishing: 'Hello World: 4'
[listener-2] [INFO] [1631185650.560414227] [listener]: I heard: [Hello World: 4]
[talker-1] [INFO] [1631185651.559902475] [talker]: Publishing: 'Hello World: 5'
[listener-2] [INFO] [1631185651.560619072] [listener]: I heard: [Hello World: 5]
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[listener-2] [INFO] [1631185651.639150181] [rclcpp]: signal_handler(signal_value=2)
[INFO] [listener-2]: process has finished cleanly [pid 117179]
[INFO] [talker-1]: process has finished cleanly [pid 117177]
[talker-1] [INFO] [1631185651.639171964] [rclcpp]: signal_handler(signal_value=2)

```
トレースポイントの確認
```
$ # 以下のトレースポイントが表示されていることを確認
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

jupyter 上で確認。サンプルは`caret_demos/samples/talker_listener`にあります。


