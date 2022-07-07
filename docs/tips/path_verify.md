# path verify tools

CARET requires some setup and configuration that users should do.
The `path.verify()` command can check the settings are completed.
If ocuring errors or warnings, please follow the error and warning messages.

## Use method

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

## Check items

- Check whether using caret-rclcpp [path.verify()]

  Target applications need to be built based on the rclcpp provided by CARET, not ROS 2.
  The 'path.verify()' command checks the based rclcpp.

  If not caret/rclcpp, the following warning is occured.

  ```bash
  WARNING : 2022-03-18 12:53:54 | 'caret/rclcpp' may not be used in subscriber of '/localization/pose_estimator/ndt_scan_matcher'.
  ```

- Check that path elements have valid message_context values
