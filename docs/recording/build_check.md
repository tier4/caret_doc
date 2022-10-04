# Build with CARET

## Build a target application with CARET

A target application should be built with CARET/rclcpp to record trace data. If you have already built the application without CARET/rclcpp, you have to build the application with CARET/rclcpp again.

For building the application with CARET/rclcpp, CARET's `local_setup.bash` should be applied along with ROS 2's `setup.bash` as shown below. Also, `-DBUILD_TESTING=OFF` should be given to build option.

```sh
cd <path-to-workspace>

source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash  # please keep the order after ROS 2's setup.bash

colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

<prettier-ignore-start>
!!!info "Reason for building a target application with CARET/rclcpp"
      CARET/rclcpp is a fork of [ROS 2-based rclcpp](https://github.com/ros2/rclcpp) which has some additional tracepoints defined by CARET.
      Some tracepoints must be added to template implementation, which is referred by rclcpp header files, for CARET to record a target application.
      In order to apply rclcpp which has the additional tracepoints, the application have to be built with CARET/rclcpp again.
      Therefore, CARET cannot trace the application provided by Ubuntu's aptitude such as `demo_nodes_cpp`.
      If you want to trace such pre-build packages, please build them again from source code.
<prettier-ignore-end>

<prettier-ignore-start>
!!!info "Reason for giving `-DBUILD_TESTING=OFF`"
      To use CARET, you need to use forked shared libraries and headers such as CARET/rclcpp.
      In the test codes, CARET/rclcpp is not available due to loading priority issues for headers.
      Depending on the version of CARET, conflicts may occur between the shared libraries of
      ros-rclcpp and the headers of CARET/rclcpp, resulting in compile errors.
      Therefore, test codes have to be excluded from building.
<prettier-ignore-end>

## Check whether CARET/rclcpp is applied to each package

You can check whether a target application is successfully built with CARET/rclcpp using `ros2 caret check_caret_rclcpp` command.

```sh
ros2 caret check_caret_rclcpp --workspace <path-to-workspace>
```

| Output Message                                                  | Description                                                                                     |
| --------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `All packages are built using caret-rclcpp`                     | There is no problem                                                                             |
| `The following packages have not been built using caret-rclcpp` | CARET/rclcpp is not applied to the listed packages<br> (Please read the next section to fix it) |

## How to fix

In case CARET/rclcpp is not applied to the package you want to analyze, you need to fix it. The followings show possible causes and solutions.

- Case 1: All packages are listed as CARET/rclcpp is not applied
  - Make sure you applied CARET's `local_setup.bash` after ROS2's `setup.bash` (keep the order)
- Case 2: Some, but not all, packages are listed as CARET/rclcpp is not applied
  - Make sure you have the following line in `package.xml` in the listed package
    - `<depend>rclcpp</depend>`

<prettier-ignore-start>
!!!info
      The listed packages are not traced while other packages built with CARET/rclcpp are properly traced. Therefore, you can ignore this message if you don't need to trace/analyze the listed packages.
<prettier-ignore-end>
