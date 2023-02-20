# Changelog

## CARET

### v0.4.4 <small>\_ Feb 21, 2022</small> {id = "0.4.4"}

- Fixed the time format of the time-series graphs ([CARET_analyze #255](https://github.com/tier4/CARET_analyze/pull/255))
- Added the design documentation for trace filtering to exclude events in DDS-layer ([CARET_doc #158](https://github.com/tier4/CARET_doc/pull/158))
- Moved `get_range()` function to `record` package ([CARET_analyze #254](https://github.com/tier4/CARET_analyze/pull/254))

### v0.4.3 <small>\_ Feb 10, 2022</small> {id = "0.4.3"}

- Fixed the bug preventing `check_ctf` from running properly after supporting activation/deactivation on runtime ([CARET_analyze #251](https://github.com/tier4/CARET_analyze/pull/251))
- Suppressed update frequency of progress bar from `tqdm` when reading trace data ([CARET_analyze #248](https://github.com/tier4/CARET_analyze/pull/248))
- Fixed incorrect annotation from `list` to `List` ([CARET_analyze #252](https://github.com/tier4/CARET_analyze/pull/252))
- Changed `setup_caret.bash` script to prompt users to install the packages ([caret #72](https://github.com/tier4/caret/pull/72))

### v0.4.2 <small>\_ Jan 23, 2022</small> {id = "0.4.2"}

- Added lightweight recording option to `ros2caret` ([ros2caret #63](https://github.com/tier4/ros2caret/pull/63))
- Removed old interface of `Plot.create_callback_jitter_plot` ([CARET_analyze #231](https://github.com/tier4/CARET_analyze/pull/231))
- Refactored `Plot.create_callback_scheduling_plot` ([CARET_analyze #240](https://github.com/tier4/CARET_analyze/pull/240))
- Added new API to combine two paths found by `Architecture.search_paths()` ([CARET_analyze #224](https://github.com/tier4/CARET_analyze/pull/224))
- Fixed the bug in `Records` class ([CARET_analyze #246](https://github.com/tier4/CARET_analyze/pull/246))
- Applied minor refactoring ([CARET_analyze #244](https://github.com/tier4/CARET_analyze/pull/244), [CARET_analyze #243](https://github.com/tier4/CARET_analyze/pull/243), [CARET_analyze #237](https://github.com/tier4/CARET_analyze/pull/237), [CARET_analyze #234](https://github.com/tier4/CARET_analyze/pull/234))
- Suppressed warning message outputted during runtime recording ([CARET_analyze #238](https://github.com/tier4/CARET_analyze/pull/238), [CARET_analyze #236](https://github.com/tier4/CARET_analyze/pull/236))
- Added `construction_order` item to identify multiple callbacks which have common symbol ([CARET_analyze #225](https://github.com/tier4/CARET_analyze/pull/225))

### v0.4.1 <small>\_ Dec 26, 2022</small> {id = "0.4.1"}

- **New**: Added functions to assign intra-node data path with `message_context` ([CARET_analyze #196](https://github.com/tier4/CARET_analyze/pull/196))
- **Update**: Changed design of visualization API for refactoring to avoid usage of Pandas ([CARET_analyze #116](https://github.com/tier4/CARET_analyze/pull/116), [CARET_analyze #167](https://github.com/tier4/CARET_analyze/pull/167), [CARET_analyze #168](https://github.com/tier4/CARET_analyze/pull/168), [CARET_analyze #170](https://github.com/tier4/CARET_analyze/pull/170))
- Added configure file to control log level to suppress warning message ([CARET_analyze #220](https://github.com/tier4/CARET_analyze/pull/220))
- Changed behavior of visualization to output empty graph if inputted data does not have any data rather than raising exception ([CARET_analyze #223](https://github.com/tier4/CARET_analyze/pull/223))
- Fixed `ValueError`, when `create_publish_subscription_frequency_plot` is called with being inputted empty object ([CARET_analyze #227](https://github.com/tier4/CARET_analyze/pull/227))
- Added metric name to tooltip for identifying plots on a graph

### v0.4.0 <small>\_ Dec 16, 2022</small> {id = "0.4.0"}

- **New**: Added a new feature to start, stop, and resume even recording whenever users want to ([CARET_trace #68](https://github.com/tier4/CARET_trace/pull/68), [CARET_analyze #190](https://github.com/tier4/CARET_analyze/pull/190), [ros2caret #48](https://github.com/tier4/ros2caret/pull/48), and [CARET_doc #126](https://github.com/tier4/CARET_doc/pull/126))

### v0.3.4 <small>\_ Dec 12, 2022</small> {id = "0.3.4"}

- **New**: Added functions to rename subsystems in Architecture object like executor, node, callback and topic ([CARET_analyze#156](https://github.com/tier4/CARET_analyze/pull/156))
- Suppressed warning messages, which are outputted because `service` event is not supported, during trace data loading ([CARET_analyze#192](https://github.com/tier4/CARET_analyze/pull/192))
- Fixed bug which calculated incorrect period or frequency ([CARET_analyze#213](https://github.com/tier4/CARET_analyze/pull/213))
- Fixed error messages from `Mypy` ([CARET_analyze#211](https://github.com/tier4/CARET_analyze/pull/211))

### v0.3.3 <small>\_ Nov 28, 2022</small> {id = "0.3.3"}

- Added guidances for a beginner to avoid getting stuck in unexpected cases ([CARET_analyze #200](https://github.com/tier4/CARET_analyze/pull/200) and [CARET_analyze #186](https://github.com/tier4/CARET_analyze/pull/186))
- Added `publishers` and `subscriptions` properties to `Architecture` and `Application` classes ([CARET_analyze #179](https://github.com/tier4/CARET_analyze/pull/179), [CARET_analyze #180](https://github.com/tier4/CARET_analyze/pull/180))
- Added function to update cache file named as `caret_converted` ([CARET_analyze#189](https://github.com/tier4/CARET_analyze/pull/189))
- Suppressed unnecessary warnings in `check_caret_rclcpp` ([ros2caret #53](https://github.com/tier4/ros2caret/pull/53), [ros2caret #54](https://github.com/tier4/ros2caret/pull/54))
- Improved documentation of installation ([CARET_doc #111](https://github.com/tier4/CARET_doc/pull/111)), configuration ([CARET_doc #112](https://github.com/tier4/CARET_doc/pull/112)), visualization ([CARET_doc #98](https://github.com/tier4/CARET_doc/pull/98)), and design ([CARET_doc#106](https://github.com/tier4/CARET_doc/pull/106))
- Fixed a bug which makes incorrect graphs ([CARET_analyze #201](https://github.com/tier4/CARET_analyze/pull/201))

### v0.3.2 <small>\_ Nov 14, 2022</small> {id = "0.3.2"}

- Improved warning messages of CARET_analyze ([CARET_analyze #144](https://github.com/tier4/CARET_analyze/pull/144), [CARET_analyze #158](https://github.com/tier4/CARET_analyze/pull/158), [CARET_analyze #162](https://github.com/tier4/CARET_analyze/pull/162), [CARET_analyze #172](https://github.com/tier4/CARET_analyze/pull/172), [CARET_analyze #182](https://github.com/tier4/CARET_analyze/pull/182))
- Added properties of `publishers` and `subscriptions` to `Application` and `Architecture` object([CARET_analyze #179](https://github.com/tier4/CARET_analyze/pull/179))
- Improved ros2caret([ros2caret #44](https://github.com/tier4/ros2caret/pull/44), [ros2caret #50](https://github.com/tier4/ros2caret/pull/50))
- Enlarged buffer size([ros2_tracing #2](https://github.com/tier4/ros2_tracing/pull/2))
- Added some minor changes ([CARET_analyze #183](https://github.com/tier4/CARET_analyze/pull/183), [CARET_analyze #187](https://github.com/tier4/CARET_analyze/pull/187))

### v0.3.1 <small>\_ Oct 31, 2022</small> {id = "0.3.1"}

- Added record function to ros2caret for upcoming feature. ([ros2caret #38](https://github.com/tier4/ros2caret/pull/38))
- Added function to display histogram for response time ([CARET_analyze #165](https://github.com/tier4/CARET_analyze/pull/165))
- Added API to calculate period and latency ([CARET_analyze #167](https://github.com/tier4/CARET_analyze/pull/167), [CARET_analyze #168](https://github.com/tier4/CARET_analyze/pull/168))
- Fixed some bugs ([CARET_analyze #141](https://github.com/tier4/CARET_analyze/pull/141), [CARET_analyze #153](https://github.com/tier4/CARET_analyze/pull/153), [CARET_analyze #157](https://github.com/tier4/CARET_analyze/pull/157))
- Refactored `Architecture` object interface ([CARET_analyze](https://github.com/tier4/CARET_analyze/pull/84))
- Added some minor changes ([CARET_analyze #139](https://github.com/tier4/CARET_analyze/pull/139), [CARET_analyze #146](https://github.com/tier4/CARET_analyze/pull/146), [CARET_analyze #147](https://github.com/tier4/CARET_analyze/pull/147), [CARET_analyze #151](https://github.com/tier4/CARET_analyze/pull/151), [CARET_analyze #159](https://github.com/tier4/CARET_analyze/pull/159), [CARET_analyze #162](https://github.com/tier4/CARET_analyze/pull/162), [CARET_analyze #165](https://github.com/tier4/CARET_analyze/pull/165), [CARET_analyze#171](https://github.com/tier4/CARET_analyze/pull/171))

### v0.3.0 <small>\_ Sept 26, 2022</small> {id = "0.3.0"}

- Supported OS: Ubuntu 22.04
- Supported ROS Dist.: ROS Humble
- Imported to Ubuntu 22.04 and ROS Humble
- Added new feature to show response time ([CARET_analyze #96](https://github.com/tier4/CARET_analyze/pull/96))
- Added new APIs to visualize frequency, period, and latency of publishes, subscriptions, and communications ([CARET_analyze #124](https://github.com/tier4/CARET_analyze/pull/124#pullrequestreview-1098527296), [#130](https://github.com/tier4/CARET_analyze/pull/130), [#133](https://github.com/tier4/CARET_analyze/pull/133), [#134](https://github.com/tier4/CARET_analyze/pull/134), [#136](https://github.com/tier4/CARET_analyze/pull/136), [#140](https://github.com/tier4/CARET_analyze/pull/140))
- Reduced memory consumption wasted for recorded data ([CARET_analyze #100](https://github.com/tier4/CARET_analyze/pull/100))
- Launched [CARET_analyze API document](https://tier4.github.io/CARET_analyze/latest/)
- Added small fixes
  - [CARET_analyze #107](https://github.com/tier4/CARET_analyze/pull/106)
  - [CARET_analyze #107](https://github.com/tier4/CARET_analyze/pull/107)
  - [CARET_analyze #108](https://github.com/tier4/CARET_analyze/pull/108)
  - [CARET_analyze #109](https://github.com/tier4/CARET_analyze/pull/109)
  - [CARET_analyze #112](https://github.com/tier4/CARET_analyze/pull/112)
  - [CARET_analyze #115](https://github.com/tier4/CARET_analyze/pull/115)

### v0.2.3 <small>\_ July 14, 2022</small> {id = "2.3.0"}

- Supported OS: Ubuntu 20.04
- Supported ROS Dist.: ROS Galactic
- Improved output message from `path.verify()` method
- Improved view of graphs
- Added wildcard support for `Architecture.callbacks()` method
- Added command line function to check trace data
- Fixed specification of `Architecture.search_paths()` method
- Added function to extract duplicated timer callback

### v0.2.2 <small>\_ May 2, 2022</small> {id = "2.2.0"}

- Supported OS: Ubuntu 20.04
- Supported ROS Dist.: ROS Galactic
- Added feature to measure complicated node path
- Added feature to choose optional trace points
- Added `Architecture.search_paths()` method
- Improved trace filtering function
