# Changelog

## CARET

### v0.4.15 <small>\_ Sep 4, 2023</small> {id = "0.4.15"}
- Updated the version of humble branch in tier4/rclcpp repository, which is forked from ros2/rclcpp, from 16.0.1 to 16.0.5. ([rclcpp #4](https://github.com/tier4/rclcpp/pull/4))
- Refactored and added APIs to calculate response time, these are a preparation to add options for visualizing graph in the future update. ([CARET_analyze #310](https://github.com/tier4/CARET_analyze/pull/310)), ([CARET_analyze #313](https://github.com/tier4/CARET_analyze/pull/313)), ([CARET_analyze #319](https://github.com/tier4/CARET_analyze/pull/319))
- Fixed typo in CMakeLists.txt. ([CARET_analyze_cpp_impl #131](https://github.com/tier4/CARET_analyze_cpp_impl/pull/131))
- Specified Pandas library version as '<2.1.0' to avoid problems with the Pandas library 2.1.0. ([CARET_analyze #324](https://github.com/tier4/CARET_analyze/pull/324))
- Fixed to force loading process to ignore the second and subsequent callbacks even if "Duplicated callback id" occurs during CARET analyzing. ([CARET_analyze #314](https://github.com/tier4/CARET_analyze/pull/314))

### v0.4.14 <small>\_ Aug 21, 2023</small> {id = "0.4.14"}

- Supported for function argument changes in `ros/ros2_tracing`. This support is in preparation for functionality improvements that will make rebuilding of the measurement application unnecessary. ([rclcpp #2](https://github.com/tier4/rclcpp/pull/2)), ([ros2_tracing #5](https://github.com/tier4/ros2_tracing/pull/5)), ([CARET_trace #136](https://github.com/tier4/CARET_trace/pull/136)), ([CARET_analyze #302](https://github.com/tier4/CARET_analyze/pull/302)), ([CARET_doc #204](https://github.com/tier4/CARET_doc/pull/204))
  - This change affects trace data compatibility. However, this only affects cases where trace data captured after this update is analyzed by caret_analyze before this update. Please see the table below for the supported combinations.

|                               | trace data (<=0.4.13) | trace data (>=v0.4.14) |
| ----------------------------- | :------------------: | :--------------------: |
| **CARET_analyze (<=v0.4.13)** |      Supported       |      Unsupported       |
| **CARET_analyze (>=v0.4.14)** |      Supported       |       Supported        |

- Adjusted Lttng buffer size to appropriate value. ([ros2_tracing #6](https://github.com/tier4/ros2_tracing/pull/6))
- Fixed a build error that occurred in environments older than Python 3.10. ([ros2caret #81](https://github.com/tier4/ros2caret/pull/81))

### v0.4.13 <small>\_ Jul 24, 2023</small> {id = "0.4.13"}

- Fixed a bug that delayed starting a measurement after `caret record` is launched depending on the time elapsed after the application being measured was launched. ([CARET_trace #129](https://github.com/tier4/CARET_trace/pull/129))

### v0.4.12 <small>\_ Jul 10, 2023</small> {id = "0.4.12"}

- Fixed a bug in the Plot class API that could not create graphs correctly when the `xaxis_type='sim_time'` option was specified. ([CARET_analyze #276](https://github.com/tier4/CARET_analyze/pull/276)), ([CARET_analyze #306](https://github.com/tier4/CARET_analyze/pull/306))
- Fixed a bug that caused incorrect warnings to be output when using `ros2 caret check_ctf`. ([CARET_analyze #308](https://github.com/tier4/CARET_analyze/pull/308))

### v0.4.11 <small>\_ Jun 26, 2023</small> {id = "0.4.11"}

- Applied to the type hinting for generics in standard collections adopted from Python 3.9. ([CARET_analyze #294](https://github.com/tier4/CARET_analyze/pull/294)), ([CARET_analyze #299](https://github.com/tier4/CARET_analyze/pull/299))
- Fixed a segmentation fault problem that occurred depending on the condition of the data analyzed by CARET_analyze. ([CARET_analyze_cpp_impl #79](https://github.com/tier4/CARET_analyze_cpp_impl/pull/79))
- Added `ros2 caret version` command to get the CARET version. ([ros2caret #69](https://github.com/tier4/ros2caret/pull/69))

### v0.4.10 <small>\_ Jun 12, 2023</small> {id = "0.4.10"}

- **Update**: Changed trace points used to tie records of inter-process communication. This change affects trace data compatibility. However, this only affects cases where trace data captured after this update is analyzed by caret_analyze before this update. In that case, update caret_analyze ([CARET_analyze #296](https://github.com/tier4/CARET_analyze/pull/296)), ([CARET_trace #113](https://github.com/tier4/CARET_trace/pull/113)), ([CARET_doc #191](https://github.com/tier4/CARET_doc/pull/191)), ([ros2caret #70](https://github.com/tier4/ros2caret/pull/70))
  - Please see the table below for the supported combinations

|                               | trace data (<=0.4.9) | trace data (>=v0.4.10) |
| ----------------------------- | :------------------: | :--------------------: |
| **CARET_analyze (<=v0.4.9)**  |      Supported       |      Unsupported       |
| **CARET_analyze (>=v0.4.10)** |      Supported       |       Supported        |

- Added remove functions to modify intra-node data path: `remove_variable_passing` and `remove_publisher_callback` ([CARET_analyze #297](https://github.com/tier4/CARET_analyze/pull/297))

### v0.4.9 <small>\_ May 15, 2023</small> {id = "0.4.9"}

- **Update**:Improved path analysis to analyze paths that contain the same symbol object ([CARET_analyze #261](https://github.com/tier4/CARET_analyze/pull/261)), ([CARET_doc #183](https://github.com/tier4/CARET_doc/pull/183))
- Improved Plot class not to plot on a graph when communication is not established ([CARET_analyze #291](https://github.com/tier4/CARET_analyze/pull/291)), ([CARET_doc #184](https://github.com/tier4/CARET_doc/pull/184))
- Removed old modules: `node_graph` and `callback_graph` ([CARET_analyze #290](https://github.com/tier4/CARET_analyze/pull/290))

### v0.4.8 <small>\_ May 1, 2023</small> {id = "0.4.8"}

- Removed deprecated interface such as `message_flow` ([CARET_analyze #179](https://github.com/tier4/CARET_doc/pull/179)), ([CARET_analyze #274](https://github.com/tier4/CARET_analyze/pull/274)), ([CARET_analyze #282](https://github.com/tier4/CARET_analyze/pull/282))
- Reduced warnings when loading trace data ([CARET_analyze #284](https://github.com/tier4/CARET_analyze/pull/284))

### v0.4.7 <small>\_ Apr 17, 2023</small> {id = "0.4.7"}

- Refactored the Plot package design ([CARET_analyze #267](https://github.com/tier4/CARET_analyze/pull/267)), ([CARET_analyze #275](https://github.com/tier4/CARET_analyze/pull/275)), ([CARET_analyze #277](https://github.com/tier4/CARET_analyze/pull/277)), ([CARET_analyze #278](https://github.com/tier4/CARET_analyze/pull/278)), ([CARET_analyze #279](https://github.com/tier4/CARET_analyze/pull/279)), ([CARET_analyze #280](https://github.com/tier4/CARET_analyze/pull/280))

### v0.4.6 <small>\_ Apr 3, 2023</small> {id = "0.4.6"}

- **New**: Added function to visualize response time with the stacked bar graph([CARET_analyze #265](https://github.com/tier4/CARET_analyze/pull/265), [CARET_analyze #259](https://github.com/tier4/CARET_analyze/pull/259), [CARET_doc #169](https://github.com/tier4/CARET_doc/pull/169))
- **Update**: Added selective path definition to include or exclude first and last callback execution ([CARET_analyze #263](https://github.com/tier4/CARET_analyze/pull/263), [CARET_analyze #271](https://github.com/tier4/CARET_analyze/pull/271), [CARET_analyze #272](https://github.com/tier4/CARET_analyze/pull/272), [CARET_analyze #273](https://github.com/tier4/CARET_analyze/pull/273))
- Added a new function to get difference between two Architecture objects ([CARET_analyze #245](https://github.com/tier4/CARET_analyze/pull/245), [CARET_doc #168](https://github.com/tier4/CARET_doc/pull/168))
- Removed deprecated interface such as `callback_sched` and `Plot.create_*_plot` ([CARET_analyze #266](https://github.com/tier4/CARET_analyze/pull/266))
- Refactored visualization API such as `message_flow` ([CARET_analyze #267](https://github.com/tier4/CARET_analyze/pull/267))

### v0.4.5 <small>\_ Mar 20, 2023</small> {id = "0.4.5"}

- Enhanced the trace filtering to exclude selected DDS events ([CARET_trace #101](https://github.com/tier4/CARET_trace/pull/101))
- Added the unit to the label of y-axis on time-series graph ([CARET_analyze #256](https://github.com/tier4/CARET_analyze/pull/256))
- Refactored the Plot package ([CARET_analyze #257](https://github.com/tier4/CARET_analyze/pull/257), [CARET_analyze #258](https://github.com/tier4/CARET_analyze/pull/258))
- Added the docstring for explaining rename methods for Architecture object ([CARET_analyze #253](https://github.com/tier4/CARET_analyze/pull/253))

### v0.4.4 <small>\_ Feb 21, 2023</small> {id = "0.4.4"}

- Fixed the time format of the time-series graphs ([CARET_analyze #255](https://github.com/tier4/CARET_analyze/pull/255))
- Added the design documentation for trace filtering to exclude events in DDS-layer ([CARET_doc #158](https://github.com/tier4/CARET_doc/pull/158))
- Moved `get_range()` function to `record` package ([CARET_analyze #254](https://github.com/tier4/CARET_analyze/pull/254))

### v0.4.3 <small>\_ Feb 10, 2023</small> {id = "0.4.3"}

- Fixed the bug preventing `check_ctf` from running properly after supporting activation/deactivation on runtime ([CARET_analyze #251](https://github.com/tier4/CARET_analyze/pull/251))
- Suppressed update frequency of progress bar from `tqdm` when reading trace data ([CARET_analyze #248](https://github.com/tier4/CARET_analyze/pull/248))
- Fixed incorrect annotation from `list` to `List` ([CARET_analyze #252](https://github.com/tier4/CARET_analyze/pull/252))
- Changed `setup_caret.bash` script to prompt users to install the packages ([caret #72](https://github.com/tier4/caret/pull/72))

### v0.4.2 <small>\_ Jan 23, 2023</small> {id = "0.4.2"}

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
