# Installation

## Requirements

CARET is confirmed to run on the platforms shown in the following table with supported version.

| ROS 2  | OS           | LTTng       | Python | Status          |
| :----- | :----------- | :---------- | :----- | :-------------- |
| Jazzy  | Ubuntu 24.04 | stable-2.13 | 3.12.x | Supported       |
| Humble | Ubuntu 22.04 | stable-2.13 | 3.10.x | Supported (LTS) |

Please follow the steps according to your ROS 2 distribution. For jazzy, ensure you run the export command in the setup step to comply with PEP 668.

## Installation

Installation using meta repository is the least time-consuming way to install CARET.  
With meta repository and Ansible, you can skip the laborious manual setup which is explained in manual installation(./manual_installation.md) section (written in Japanese).

Please execute the following steps on Ubuntu 22.04 or 24.04. The order is important so that you have to follow the steps in order.

   <details>
   <summary>for jazzy</summary>

Since Ubuntu 24.04 (Jazzy) restricts pip installations into the system environment (PEP 668), you need to acknowledge this by setting an environment variable(PIP_BREAK_SYSTEM_PACKAGES) before running the setup script.
If you are using Jazzy, please ensure you follow the steps under the "jazzy" tabs below, which include the necessary environment variable.

   </details>

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

=== "humble"

    ``` bash
    mkdir src
    vcs import src < caret.repos
    ```

=== "iron"

    ``` bash
    mkdir src
    vcs import src < caret_iron.repos
    ```

=== "jazzy"

    ``` bash
    mkdir src
    vcs import src < caret_jazzy.repos
    ```

<prettier-ignore-start>
3. Run `setup_caret.sh`.
<prettier-ignore-end>

=== "humble"

    ``` bash
    ./setup_caret.sh
    ```

=== "iron"

    ``` bash
    ./setup_caret.sh -d iron
    ```

=== "jazzy"

    ``` bash
    export PIP_BREAK_SYSTEM_PACKAGES=1
    ./setup_caret.sh -d jazzy
    ```

<prettier-ignore-start>
4. Build the workspace.
<prettier-ignore-end>

=== "humble"

    ``` bash
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

=== "iron"

    ``` bash
    source /opt/ros/iron/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

=== "jazzy"

    ``` bash
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

<prettier-ignore-start>
5. Check whether CARET (ros2-tracing) is enabled.
<prettier-ignore-end>

CARET inherits some functions from [ros2-tracing](https://gitlab.com/ros-tracing/ros2_tracing).

```bash
source ~/ros2_caret_ws/install/local_setup.bash
ros2 run tracetools status # return Tracing enabled
```

If you see `Tracing enabled`, you can continue to apply CARET to your application.
