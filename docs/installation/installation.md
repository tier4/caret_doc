# Installation

## Requirements

CARET is confirmed to run on the platforms shown in the following table with supported version.

| ROS 2  | OS           | LTTng       | Python | Status          |
| :----- | :----------- | :---------- | :----- | :-------------- |
| Jazzy  | Ubuntu 24.04 | stable-2.13 | 3.12.x | Supported       |
| Humble | Ubuntu 22.04 | stable-2.13 | 3.10.x | Supported (LTS) |

> **Note:** This document describes for Jazzy. If you are using Humble, please refer to the documentation for v0.7.3 ([v0.7.3 installation](https://tier4.github.io/caret_doc/v0.7.3/installation/installation/)).

## Installation

Installation using meta repository is the least time-consuming way to install CARET.

Since Ubuntu 24.04 restricts pip installations into the system environment (PEP 668), you need to acknowledge this by setting an environment variable(PIP_BREAK_SYSTEM_PACKAGES) before running the setup script.

<prettier-ignore-start>
1. Clone `caret` and enter the directory.
<prettier-ignore-end>

```bash
git clone https://github.com/tier4/caret.git ros2_caret_ws
cd ros2_caret_ws
```

<prettier-ignore-start>
2. Create the src directory and clone repositories into it.
<prettier-ignore-end>

CARET uses vcstool to construct workspaces.

=== "Jazzy"

    ``` bash
    mkdir src
    vcs import src < caret.repos
    ```

=== "Humble"

    ``` bash
    mkdir src
    vcs import src < caret_humble.repos
    ```

<prettier-ignore-start>
3. Run `setup_caret.sh`.
<prettier-ignore-end>

=== "Jazzy"

    ``` bash
    export PIP_BREAK_SYSTEM_PACKAGES=1
    ./setup_caret.sh
    ```

=== "Humble"

    ``` bash
    ./setup_caret.sh -d humble
    ```

<prettier-ignore-start>
4. Build the workspace.
<prettier-ignore-end>

=== "Jazzy"

    ``` bash
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

=== "Humble"

    ``` bash
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

<prettier-ignore-start>
5. Check whether CARET (ros2-tracing) is enabled.
<prettier-ignore-end>

```bash
$ source ~/ros2_caret_ws/install/local_setup.bash
$ ros2 run tracetools status
Tracing enabled # return value
```

If you see `Tracing enabled`, you can continue to apply CARET to your application.
