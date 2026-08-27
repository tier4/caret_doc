<!-- cspell:ignore Drclcpp Dtracetools ltracetools -->

# Known issues

## Install

### Warnings caused by `setup.py`

- Issue
  - The following warnings happen when building CARET

```sh
/usr/local/lib/python3.10/dist-packages/setuptools/command/develop.py:40: EasyInstallDeprecationWarning: easy_install command is deprecated.
!!

        ********************************************************************************
        Please avoid running ``setup.py`` and ``easy_install``.
        Instead, use pypa/build, pypa/installer or other
        standards-based tools.

        See https://github.com/pypa/setuptools/issues/917 for details.
        ********************************************************************************

!!
  easy_install.initialize_options(self)
```

```sh
/usr/local/lib/python3.10/dist-packages/setuptools/_distutils/cmd.py:66: SetuptoolsDeprecationWarning: setup.py install is deprecated.
!!

        ********************************************************************************
        Please avoid running ``setup.py`` directly.
        Instead, use pypa/build, pypa/installer or other
        standards-based tools.

        See https://blog.ganssle.io/articles/2021/10/setup-py-deprecated.html for details.
        ********************************************************************************

!!
  self.initialize_options()
```

- Cause
  - Python3 deprecates building with `setup.py`
  - However, `setup.py` is used to build ROS 2 by colcon
- Workaround
  - This warning does not prevent CARET from working and does not require any action
  - However, if you really want to remove the warning, the following steps can be taken to suppress the warning

```sh
export PYTHONWARNINGS=ignore:"setup.py install is deprecated.",ignore:"easy_install command is deprecated."
```

## Recording

### Only metadata is recorded

- Issue
  - Only metadata file (`~/.ros/tracing/ooo/ust/uid/1000/64-bit/metadata`) is created, and the size of the trace data is always about 28 KByte
- Cause
  - The size of LTTng buffer is too large for some environments
  - It happens especially when
    - Recording on Docker
    - and the number of CPUs is large (e.g. 24 cores)
- Workaround
  - Decrease the number of the LTTng buffer size
    - [lttng_impl.py](https://github.com/tier4/ros2_tracing/blob/64545052077d38c770b0c6e73fad221bcaba0583/tracetools_trace/tracetools_trace/tools/lttng_impl.py#L157)
      - Rebuild is not required after the modification
    - Please note that decreasing the buffer size may cause tracer discarded
