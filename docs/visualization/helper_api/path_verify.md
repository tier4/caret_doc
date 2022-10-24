# Error detection using `path.verify()`

CARET requires some setup and configuration that users should do.
The function `path.verify()` can check that the settings are completed.
If there are some problems of settings, this function shows hints to resolve them.

## Usage

```python
from caret_analyze import Architecture, Application, Lttng

# read architecture file.
arch = Architecture('yaml', '/path/to/architecture_file.yaml')

# read trace result.
lttng = Lttng('/path/to/trace_data')

# map architecture information to trace result.
app = Application(arch, lttng)

path = app.get_path('target_path')
path.verify()
```

## What `path.verify()` checks

- Check whether using caret/rclcpp [path.verify()]

  Your applications need to be built with `rclcpp` provided by CARET, not ROS 2.
  The function `path.verify()` checks which `rclcpp` is used for building your application.

  If caret/rclcpp is not applied, the following warning message is outputted on your terminal or Jupyter notebook.

  ```bash
  WARNING : 2022-03-18 12:53:54 | 'caret/rclcpp' may not be used in subscriber of '/localization/pose_estimator/ndt_scan_matcher'.
  ```

- Check whether node latency can be calculated with referring to `message_context`
