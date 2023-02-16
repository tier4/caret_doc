# Known issues

## Build

### Conflicts of dependency on `libtracetools.so`

- Issue
  - The following errors happen when building a target application with certain packages like `pcl_ros`
    - ``undefined reference to `ros_trace_message_construct'``
    - ``undefined reference to `ros_trace_rclcpp_intra_publish'``
    - ``undefined reference to `ros_trace_dispatch_subscription_callback'``
    - and so on
- Cause
  - `~/ros2_caret_ws/install/tracetools/lib/libtracetools.so` needs to be linked, but `/opt/ros/humble/lib/libtracetools.so` is referred when using some packages
  - For instance, `pcl_ros` package has `/opt/ros/humble/share/pcl_ros/cmake/export_pcl_rosExport.cmake` which enforces `/opt/ros/humble/lib/libtracetools.so` to be linked
- Workaround
  - Remove `/opt/ros/humble/lib/libtracetools.so;` from `/opt/ros/humble/share/pcl_ros/cmake/export_pcl_rosExport.cmake`

```sh
sudo cp /opt/ros/humble/share/pcl_ros/cmake/export_pcl_rosExport.cmake /opt/ros/humble/share/pcl_ros/cmake/export_pcl_rosExport.cmake.bak
sudo sed -i -e 's/\/opt\/ros\/humble\/lib\/libtracetools.so;//g' /opt/ros/humble/share/pcl_ros/cmake/export_pcl_rosExport.cmake
```

### Build using ament_cmake

- Issue
  - The following error happens when building a target application using ament_cmake
    - `error: too few arguments to function ‘void ros_trace_rclcpp_publish`
- Cause
  - `SYSTEM` is added as dependencies in ament_cmake_auto by [this PR](https://github.com/ament/ament_cmake/commit/799183ab9bcfd9b66df0de9b644abaf8c9b78e84). As a result, ros2/rclcpp is used rather than CARET/rclcpp in some packages
- Workaround
  - Remove `SYSTEM` from dependencies in ament_cmake_auto

```sh
cd /opt/ros/humble/share/ament_cmake_auto/cmake/
sudo cp ament_auto_add_executable.cmake ament_auto_add_executable.cmake.bak
sudo cp ament_auto_add_library.cmake ament_auto_add_library.cmake.bak
sudo sed -i -e 's/SYSTEM//g' ament_auto_add_executable.cmake
sudo sed -i -e 's/SYSTEM//g' ament_auto_add_library.cmake
```

## Recording

### Only metadata is recorded

- Issue
  - Only metadata file (`~/.ros/tracing/ooo/ust/uid/1000/64-bit/metadata`) is created, and the size of the trace data is always about 28 KByte
- Cause
  - The size of LTTng buffer is too large for some environments
  - It happens especially when
    - recording on Docker
    - and the number of CPUs is many (e.g. 24 cores)
- Workaround
  - Decrease the number of the LTTng buffer size
    - [lttng_impl.py](https://github.com/tier4/ros2_tracing/blob/64545052077d38c770b0c6e73fad221bcaba0583/tracetools_trace/tracetools_trace/tools/lttng_impl.py#L157)
    - Please note that decreasing the buffer size may cause tracer discarded
