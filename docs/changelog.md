# Changelog

## CARET

### v0.5.9 <small>\_ Aug 7, 2024</small> {id = "0.5.9"}

- **Fix**: Temporarily limit mypy version for CI execution. ([caret_analyze #519](https://github.com/tier4/caret_analyze/pull/519))

- **Update**: Supports the implementation that users explicitly take messages using subscription->take. This allows visualization of the message flow of the path containing the nodes implemented above. ([caret_analyze #516](https://github.com/tier4/caret_analyze/pull/516))

### v0.5.8 <small>\_ Jul 29, 2024</small> {id = "0.5.8"}

- **New**: Add cppcheck to ci. At the same time, the correction of the indicated items in cppcheck is carried out. ([caret_trace #291](https://github.com/tier4/caret_trace/pull/291), [caret_analyze_cpp_impl #211](https://github.com/tier4/caret_analyze_cpp_impl/pull/211))

- **Update**: Remove the pull-request trigger in cppcheck-all-files workflow. ([caret_trace #294](https://github.com/tier4/caret_trace/pull/294))

- **Update**: Explicitly include some standard headers to support g++13. ([caret_trace #293](https://github.com/tier4/caret_trace/pull/293))

- **Fix**: Fixed a PlantUML bug in documentation. ([caret_doc #348](https://github.com/tier4/caret_doc/pull/348))

- **Update**: Reviewed import statements not at the beginning of caret_analyze. ([caret_analyze #510](https://github.com/tier4/caret_analyze/pull/510/))

- **Update**: Added feature to verify trace points on dds layer. ([caret_analyze #514](https://github.com/tier4/caret_analyze/pull/514))

- **Update**: Update docker image used for ci. ([caret_analyze #515](https://github.com/tier4/caret_analyze/pull/515), [ros2caret #194](https://github.com/tier4/ros2caret/pull/194))

- **Update**: Fixed to recognize callback groups that are not associated with the executor. ([caret_analyze #511](https://github.com/tier4/caret_analyze/pull/511))

### v0.5.7 <small>\_ Jun 24, 2024</small> {id = "0.5.7"}

- **Update**: controlling the dds_bind_addr_to_stamp output of `generic_publish()`. ([caret_trace #288](https://github.com/tier4/caret_trace/pull/288))

- **Update**: Build support for caret_analyze_cpp_impl using gcc13. Fixed build failure due to some C++ standard library header changes. ([caret_analyze_cpp_impl #210](https://github.com/tier4/caret_analyze_cpp_impl/pull/210))

- **Update**: changed the log level when the executor failed to load. ([caret_analyze #509](https://github.com/tier4/caret_analyze/pull/509))

- **Update**: fixed incorrect arguments in rclcpp_service_callback_added. ([caret_trace #289](https://github.com/tier4/caret_trace/pull/289))

- **Update**: Unify the description of doxygen comments for methods of the same family declared in tracing_controller.hpp. ([caret_trace #287](https://github.com/tier4/caret_trace/pull/287))

### v0.5.6 <small>\_ May 24, 2024</small> {id = "0.5.6"}

- **Update**: Fix command to install packages needed to build mkdocs. ([caret_doc #334](https://github.com/tier4/caret_doc/pull/344))

- **Fix**: If there is a Subscription callback with the same symbol, it will not be possible to analyze correctly. ([caret_analyze #460](https://github.com/tier4/caret_analyze/pull/460))

- **Update**: If Autoware is started and CARET measurement is started at the same time, some nodes may not be recorded. ([caret_doc #343](https://github.com/tier4/caret_doc/pull/343))

- **Fix**: Fixed a bug in NodeStructValue.get_timer (external API). ([caret_analyze #502](https://github.com/tier4/caret_analyze/pull/502))

- **Update**: Removed special duplicate check when recording trace events. ([caret_trace #284](https://github.com/tier4/caret_trace/pull/284))

- **Fix**: Fixed problem with ros2 caret check_ctf not validating even if a single path is entered. ([caret_analyze #503](https://github.com/tier4/caret_analyze/pull/503))

- **Update**: Added output of message_construct trace points during light measurements. ([ros2caret #187](https://github.com/tier4/ros2caret/pull/187))

- **Fix**: Fix for segmentation fault occurrence in autoware pandar_node_container (segmentation fault due to race condition). ([caret_trace #285](https://github.com/tier4/caret_trace/pull/285))

- **Update**: Provide a filter function suitable for analyzing multiple data. ([caret_analyze #504](https://github.com/tier4/caret_analyze/pull/504))

### v0.5.5 <small>\_ May 10, 2024</small> {id = "0.5.5"}

- **New**: Added the feature to measure systems operating with multiple hosts and to analyze the measured data. ([caret_trace #262](https://github.com/tier4/caret_trace/pull/262), [caret_analyze #445](https://github.com/tier4/caret_analyze/pull/455), [ros2caret #163](https://github.com/tier4/ros2caret/pull/163), [caret_doc #333](https://github.com/tier4/caret_doc/pull/333))

### v0.5.4 <small>\_ May 7, 2024</small> {id = "0.5.4"}

- **Fix**: Trace points of nodes specified by the environment variable CARET_IGNORE_NODES may not be excluded. ([caret_trace #280](https://github.com/tier4/caret_trace/pull/280))

- **Fix**: Bug fix for address duplication (process ID support). ([caret_trace #280](https://github.com/tier4/caret_trace/pull/280))

- **Fix**: Filtering does not work in some cases when using FastDDS in iron environment. ([caret_trace #280](https://github.com/tier4/caret_trace/pull/280))

- **Fix**: construct_static_executor is not outputting when measurement is started after the application is launched. ([caret_trace #280](https://github.com/tier4/caret_trace/pull/280))

### v0.5.3 <small>\_ Apr 22, 2024</small> {id = "0.5.3"}

- **Fix**: Removed duplicate code in processing service callbacks. No change in functionality. ([caret_analyze #494](https://github.com/tier4/caret_analyze/pull/494))

- **Update**: Removed unused objects. Record and Records class are not currently used in CARET. ([caret_analyze #493](https://github.com/tier4/caret_analyze/pull/493))

- **Update**: Unified the method of specifying modules and paths when import a module. ([caret_analyze #496](https://github.com/tier4/caret_analyze/pull/496))

- **Update**: Replaced deprecated method with currently recommended one. ([caret_analyze #497](https://github.com/tier4/caret_analyze/pull/497))

### v0.5.2 <small>\_ Mar 26, 2024</small> {id = "0.5.2"}

- **Update**: If pydantic is not imported using `type_check_decorator`, issue a warning message. ([caret_analyze #484](https://github.com/tier4/caret_analyze/pull/484))

- **Update**: Update `histogram` images in the documentation. ([caret_doc #336](https://github.com/tier4/caret_doc/pull/336))

- **Update**: Update docker images used for CI. ([caret_analyze #483](https://github.com/tier4/caret_analyze/pull/483), [ros2caret #178](https://github.com/tier4/ros2caret/pull/178))

- **Update**: Fixed a problem where the filter function would not work when the publisher_handle address changed during execution. ([caret_trace #277](https://github.com/tier4/caret_trace/pull/277))

- **Update**: Fixed annotations of API documentation. ([caret_analyze #488](https://github.com/tier4/caret_analyze/pull/488), [caret_analyze #490](https://github.com/tier4/caret_analyze/pull/490))

- **Update**: Changed to ignore some data used to calculate response time. Data present at the beginning of the measurement data that cannot be used to calculate an accurate response time is no longer used in the response time calculation. ([caret_analyze #486](https://github.com/tier4/caret_analyze/pull/486))

- **Update**: Refactoring of the code related to the `type_check_decorator`. ([caret_analyze #489](https://github.com/tier4/caret_analyze/pull/489))

- **Update**: Refactoring of the code related to the `histogram`. ([caret_analyze #481](https://github.com/tier4/caret_analyze/pull/481))

- **Update**: Improved type_check_decorator warning. When a variable-length argument violates a type, the message now includes the index of the violating location. ([caret_analyze #485](https://github.com/tier4/caret_analyze/pull/485))

- **New**: Add documentation for installation of CARET Debian packages. ([caret_doc #335](https://github.com/tier4/caret_doc/pull/335))

- **New**: Add a trace point filtering feature for the specified process. ([caret_trace #273](https://github.com/tier4/caret_trace/pull/273))

### v0.5.1 <small>\_ Mar 11, 2024</small> {id = "0.5.1"}

- **Update**: Remove the description of response time histogram. ([caret_doc #331](https://github.com/tier4/caret_doc/pull/331))

- **Fix**: Fixed that overlapping histograms are hard to see. ([caret_analyze #475](https://github.com/tier4/caret_analyze/pull/475))

- **Fix**: Fixed problem with ros2 caret command not working when built without `--symlink-install`. ([ros2caret #175](https://github.com/tier4/ros2caret/pull/175))

- **Update**: Add a dynamic type check method for the plot API. ([caret_analyze #472](https://github.com/tier4/caret_analyze/pull/472))

- **Fix**: Speeding up by limiting registration to init_events because id_remapper takes a long time to load measurement data. ([caret_analyze #480](https://github.com/tier4/caret_analyze/pull/480))

- **Fix**: Fixed warning "More than three callbacks are registered in one subscription_handle. Skip loading callback info." when parsing certain trace data. ([caret_analyze #470](https://github.com/tier4/caret_analyze/pull/470))

- **Fix**: Fixed a bug where the count of filtered data wasn't correctly displayed ("filtered to 0 events") when loading trace data. ([caret_analyze #480](https://github.com/tier4/caret_analyze/pull/480))

- **Update**: Update latency visualization API features in the Gallery section. ([caret_doc #332](https://github.com/tier4/caret_doc/pull/332))

- **Fix**: Speeding up `search_paths` by limiting number of `construction_order`. ([caret_analyze #477](https://github.com/tier4/caret_analyze/pull/477))

- **New**: Added max_construction_order argument to commands using Architecture-API. ([ros2caret #173](https://github.com/tier4/ros2caret/pull/173))

### v0.4.25 <small>\_ Feb 26, 2024</small> {id = "0.4.25"}

- **Fix**: Fixed annotations in the Plot API about TimeSeriesTypes. ([caret_analyze #463](https://github.com/tier4/caret_analyze/pull/463))

- **Fix**: Fixed an issue where the histogram would exit with an error if an empty records was entered. ([caret_analyze #466](https://github.com/tier4/caret_analyze/pull/466))

- **Update**: Added and fixed doc string information in value_object package. ([caret_analyze #454](https://github.com/tier4/caret_analyze/pull/454))

- **Update**: Copyright change from ISP to TIER IV, Inc. ([caret_analyze #473](https://github.com/tier4/caret_analyze/pull/473))

- **Update**: Removed Galactic and Ubuntu 20.04 support from the documentation. ([caret_doc #328](https://github.com/tier4/caret_doc/pull/328))

- **New**: Add a workflow to automatically update package.xml on git tag. ([caret_analye #467](https://github.com/tier4/caret_analyze/pull/467), [caret_analyze_cpp_impl #194](https://github.com/tier4/caret_analyze_cpp_impl/pull/194), [caret_trace #268](https://github.com/tier4/caret_trace/pull/268), [ros2caret #169](https://github.com/tier4/ros2caret/pull/169))

### v0.4.24 <small>\_ Jan 15, 2024</small> {id = "0.4.24"}

- **Update**: Delete past implementations of histograms and response times that are no longer needed. ([caret_analyze #442](https://github.com/tier4/caret_analyze/pull/442))

- **Fix**: Fixed behavior of `ros2 caret check_caret_rclcpp` command after iron. ([ros2caret #135](https://github.com/tier4/ros2caret/pull/135))

- **Fix**: Fixed a bug in the `ros2 caret record --immediate` option. It caused an error in the input operation if there was no standard input. ([ros2caret #164](https://github.com/tier4/ros2caret/pull/164))

- **Update**: Change in the method of determining ROS Distributions. ([caret_analyze #452](https://github.com/tier4/caret_analyze/pull/452), [ros2caret #161](https://github.com/tier4/ros2caret/pull/161))

### v0.4.23 <small>\_ Dec 12, 2023</small> {id = "0.4.23"}

- **Update**: In mypy 1.x, it is recommended to use the --check-untyped-defs option for stricter type checking. this option has been added to pytest in caret_analyze, and the warning that occurred after the option was added has been corrected. ([caret_analyze #441](https://github.com/tier4/caret_analyze/pull/441))

- **Update**: Changed implementation so that BokehUserWarning (out of range integer) in Bokeh 3.x is not generated. ([caret_analyze #447](https://github.com/tier4/caret_analyze/pull/447))

- **Update**: Fixed API list (doc_string) except value_objects. ([caret_analyze #446](https://github.com/tier4/caret_analyze/pull/446/files))

- **Fix**: Fixed argument error passed to \_find_node_path. ([caret_analyze #453](https://github.com/tier4/caret_analyze/pull/453))

- **New**: Added setup script for iron. Added build procedure and tutorial for iron. ([caret #175](https://github.com/tier4/caret/pull/175), [caret_doc #207](https://github.com/tier4/caret_doc/pull/207), [caret_doc #322](https://github.com/tier4/caret_doc/pull/322))

### v0.4.22 <small>\_ Dec 12, 2023</small> {id = "0.4.22"}

- **Update**: Support for analysis of trace data without `--light` option for applications using GenericPublisher, GenericSubscription. ([caret_analyze #432](https://github.com/tier4/caret_analyze/pull/432), [caret_analyze #445](https://github.com/tier4/caret_analyze/pull/445))

- **Update**: Fixed a bug when address duplication occurs. ([caret_analyze #342](https://github.com/tier4/caret_analyze/pull/342))

- **Update**: Update some API to support sim_time. ([caret_analyze #433](https://github.com/tier4/caret_analyze/pull/433), [ros2caret #152](https://github.com/tier4/ros2caret/pull/152))

### v0.4.21 <small>\_ Nov 27, 2023</small> {id = "0.4.21"}

- **Update**: Update maintainer. ([caret_analyze #422](https://github.com/tier4/caret_analyze/pull/422), [caret_analyze_cpp_impl #172](https://github.com/tier4/caret_analyze_cpp_impl/pull/172), [caret_trace #244](https://github.com/tier4/caret_trace/pull/244), [ros2caret #143](https://github.com/tier4/ros2caret/pull/143))

- **New**: Add trace points to GenericPublisher and GenericSubscription. ([rclcpp #7](https://github.com/tier4/rclcpp/pull/7), [caret_trace #220](https://github.com/tier4/caret_trace/pull/220), [caret_trace #228](https://github.com/tier4/caret_trace/pull/228), [caret_doc #309](https://github.com/tier4/caret_doc/pull/309))

- **Update**: Fix for mypy error in CARET_analyze PR. Support for mypy v1.x. ([caret_analyze #425](https://github.com/tier4/caret_analyze/pull/425))

- **Update**: Changed Y-axis from probability to number of samples for better readability of graphs. ([caret_analyze #430](https://github.com/tier4/caret_analyze/pull/430), [caret_doc #313](https://github.com/tier4/caret_doc/pull/313))

### v0.4.20 <small>\_ Nov 13, 2023</small> {id = "0.4.20"}

- **Update**: Update documentation with changes to ros2 caret command. ([caret_doc #295](https://github.com/tier4/caret_doc/pull/295))

- **Update**: Update tracepoint event 'caret_init'. ([caret_doc #304](https://github.com/tier4/caret_doc/pull/304))

- **Update**: Support for bokeh 3.x. ([caret_analyze #391](https://github.com/tier4/caret_analyze/pull/391))

- **Update**: Change in the method of 'Timeseries'. The method of calculating ResponseTime for the best and worst-with-external-latency cases has been changed to be the same as for `all` and `worst` case. ([caret_analyze #323](https://github.com/tier4/caret_analyze/pull/323))

- **New**: Changed the CI for caret_analyze to use cart_analyze_cpp_impl. ([caret_analyze #405](https://github.com/tier4/caret_analyze/pull/405))

- **Update**: Unify graph captions for `path`. ([caret_analyze #410](https://github.com/tier4/caret_analyze/pull/410))

### v0.4.19 <small>\_ Oct 31, 2023</small> {id = "0.4.19"}

- **Update**: Improvement of option specification of ros2 caret command. ([ros2caret #129](https://github.com/tier4/ros2caret/pull/129)), ([caret_doc #287](https://github.com/tier4/caret_doc/pull/287)), ([caret #176](https://github.com/tier4/caret/pull/176))

- **Update**: Version update support for multimethod. ([caret_analyze #403](https://github.com/tier4/caret_analyze/pull/403)), ([caret_analyze #398](https://github.com/tier4/caret_analyze/pull/398))

### v0.4.18 <small>\_ Oct 16, 2023</small> {id = "0.4.18"}

- **New**: Supported for iron tracepoint. ([caret_analyze #318](https://github.com/tier4/CARET_analyze/pull/318)), ([caret_trace #150](https://github.com/tier4/CARET_trace/pull/150)), ([ros2caret #84](https://github.com/tier4/ros2caret/pull/84)), ([ros2caret #86](https://github.com/tier4/ros2caret/pull/86))
  - Humble trace data and its analysis will still be available in later versions.
- New: Added immediate recoding option to `ros2caret`. ([ros2caret #123](https://github.com/tier4/ros2caret/pull/123))
- **New**: Added an option to ros2caret to specify the size of the Lttng buffer. ([ros2caret #85](https://github.com/tier4/ros2caret/pull/85))

### v0.4.17 <small>\_ Oct 03, 2023</small> {id = "0.4.17"}

- **Update**: Improved `case` option for calculating the elapsed time specified for functions that graph response time. ([caret_analyze #339](https://github.com/tier4/caret_analyze/pull/339)), ([caret_analyze #355](https://github.com/tier4/caret_analyze/pull/355))
  - Added new "all" option, which uses all input times in the same cycle to calculate the elapsed time.
  - Renamed "worst" option to "worst-with-external-latency".
  - Added new "worst" option, which uses the worst of all input times in the same cycle to calculate the elapsed time.

| CARET<0.4.17 |         CARET>=v0.4.7         |
| :----------: | :---------------------------: |
|     N/A      |             "all"             |
|    "best"    |            "best"             |
|     N/A      |            "worst"            |
|   "worst"    | "worst-with-external-latency" |

- **Update**: Added `create_response_time_histogram_plot` function to graph response time as histogram. ([caret_analyze #349](https://github.com/tier4/caret_analyze/pull/349))
- **Fix**: Improved measurement leaks in the first measurement after the host machine has been started up. ([caret #142](https://github.com/tier4/caret/pull/142))
- Removed unnecessary dependent modules from caret_analyze_cpp_impl. ([caret_analyze_cpp_impl #142](https://github.com/tier4/caret_analyze_cpp_impl/pull/142)), ([caret_analyze #332](https://github.com/tier4/caret_analyze/pull/332))

### v0.4.16.1 <small>\_ Sep 21, 2023</small> {id = "0.4.16.1"}

- **Fix**: Fixed an issue that prevented CARETmeasurement when using cyclonedds 0.10.x. ([caret #121](https://github.com/tier4/caret/pull/121))
- **New**: Added API to plot the response time of Paths as timeseries. ([caret_analyze #322](https://github.com/tier4/caret_analyze/pull/322))
- Changed to temporarily specify the version of `setuptools` to be installed, as the latest `setuptools` causes error during CARET building. ([caret_analyze #330](https://github.com/tier4/caret_analyze/pull/330))

### v0.4.15 <small>\_ Sep 4, 2023</small> {id = "0.4.15"}

- Updated the version of humble branch in tier4/rclcpp repository, which is forked from ros2/rclcpp, from 16.0.1 to 16.0.5. ([rclcpp #4](https://github.com/tier4/rclcpp/pull/4))
- Refactored and added APIs to calculate response time, these are a preparation to add options for visualizing graph in the future update. ([caret_analyze #310](https://github.com/tier4/caret_analyze/pull/310)), ([caret_analyze #313](https://github.com/tier4/caret_analyze/pull/313)), ([caret_analyze #319](https://github.com/tier4/caret_analyze/pull/319))
- Fixed typo in CMakeLists.txt. ([caret_analyze_cpp_impl #131](https://github.com/tier4/caret_analyze_cpp_impl/pull/131))
- Specified Pandas library version as '<2.1.0' to avoid problems with the Pandas library 2.1.0. ([caret_analyze #324](https://github.com/tier4/caret_analyze/pull/324))
- Fixed to force loading process to ignore the second and subsequent callbacks even if "Duplicated callback id" occurs during CARET analyzing. ([caret_analyze #314](https://github.com/tier4/caret_analyze/pull/314))

### v0.4.14 <small>\_ Aug 21, 2023</small> {id = "0.4.14"}

- Supported for function argument changes in `ros/ros2_tracing`. This support is in preparation for functionality improvements that will make rebuilding of the measurement application unnecessary. ([rclcpp #2](https://github.com/tier4/rclcpp/pull/2)), ([ros2_tracing #5](https://github.com/tier4/ros2_tracing/pull/5)), ([caret_trace #136](https://github.com/tier4/caret_trace/pull/136)), ([caret_analyze #302](https://github.com/tier4/caret_analyze/pull/302)), ([caret_doc #204](https://github.com/tier4/caret_doc/pull/204))
  - This change affects trace data compatibility. However, this only affects cases where trace data captured after this update is analyzed by caret_analyze before this update. Please see the table below for the supported combinations.

|                               | trace data (<=0.4.13) | trace data (>=v0.4.14) |
| ----------------------------- | :-------------------: | :--------------------: |
| **caret_analyze (<=v0.4.13)** |       Supported       |      Unsupported       |
| **caret_analyze (>=v0.4.14)** |       Supported       |       Supported        |

- Adjusted Lttng buffer size to appropriate value. ([ros2_tracing #6](https://github.com/tier4/ros2_tracing/pull/6))
- Fixed a build error that occurred in environments older than Python 3.10. ([ros2caret #81](https://github.com/tier4/ros2caret/pull/81))

### v0.4.13 <small>\_ Jul 24, 2023</small> {id = "0.4.13"}

- Fixed a bug that delayed starting a measurement after `caret record` is launched depending on the time elapsed after the application being measured was launched. ([caret_trace #129](https://github.com/tier4/caret_trace/pull/129))

### v0.4.12 <small>\_ Jul 10, 2023</small> {id = "0.4.12"}

- Fixed a bug in the Plot class API that could not create graphs correctly when the `xaxis_type='sim_time'` option was specified. ([caret_analyze #276](https://github.com/tier4/caret_analyze/pull/276)), ([caret_analyze #306](https://github.com/tier4/caret_analyze/pull/306))
- Fixed a bug that caused incorrect warnings to be output when using `ros2 caret check_ctf`. ([caret_analyze #308](https://github.com/tier4/caret_analyze/pull/308))

### v0.4.11 <small>\_ Jun 26, 2023</small> {id = "0.4.11"}

- Applied to the type hinting for generics in standard collections adopted from Python 3.9. ([caret_analyze #294](https://github.com/tier4/caret_analyze/pull/294)), ([caret_analyze #299](https://github.com/tier4/caret_analyze/pull/299))
- Fixed a segmentation fault problem that occurred depending on the condition of the data analyzed by caret_analyze. ([caret_analyze_cpp_impl #79](https://github.com/tier4/caret_analyze_cpp_impl/pull/79))
- Added `ros2 caret version` command to get the CARET version. ([ros2caret #69](https://github.com/tier4/ros2caret/pull/69))

### v0.4.10 <small>\_ Jun 12, 2023</small> {id = "0.4.10"}

- **Update**: Changed trace points used to tie records of inter-process communication. This change affects trace data compatibility. However, this only affects cases where trace data captured after this update is analyzed by caret_analyze before this update. In that case, update caret_analyze ([caret_analyze #296](https://github.com/tier4/caret_analyze/pull/296)), ([caret_trace #113](https://github.com/tier4/caret_trace/pull/113)), ([caret_doc #191](https://github.com/tier4/caret_doc/pull/191)), ([ros2caret #70](https://github.com/tier4/ros2caret/pull/70))
  - Please see the table below for the supported combinations

|                               | trace data (<=0.4.9) | trace data (>=v0.4.10) |
| ----------------------------- | :------------------: | :--------------------: |
| **caret_analyze (<=v0.4.9)**  |      Supported       |      Unsupported       |
| **caret_analyze (>=v0.4.10)** |      Supported       |       Supported        |

- Added remove functions to modify intra-node data path: `remove_variable_passing` and `remove_publisher_callback` ([caret_analyze #297](https://github.com/tier4/caret_analyze/pull/297))

### v0.4.9 <small>\_ May 15, 2023</small> {id = "0.4.9"}

- **Update**:Improved path analysis to analyze paths that contain the same symbol object ([caret_analyze #261](https://github.com/tier4/caret_analyze/pull/261)), ([caret_doc #183](https://github.com/tier4/caret_doc/pull/183))
- Improved Plot class not to plot on a graph when communication is not established ([caret_analyze #291](https://github.com/tier4/caret_analyze/pull/291)), ([caret_doc #184](https://github.com/tier4/caret_doc/pull/184))
- Removed old modules: `node_graph` and `callback_graph` ([caret_analyze #290](https://github.com/tier4/caret_analyze/pull/290))

### v0.4.8 <small>\_ May 1, 2023</small> {id = "0.4.8"}

- Removed deprecated interface such as `message_flow` ([caret_analyze #179](https://github.com/tier4/caret_doc/pull/179)), ([caret_analyze #274](https://github.com/tier4/caret_analyze/pull/274)), ([caret_analyze #282](https://github.com/tier4/caret_analyze/pull/282))
- Reduced warnings when loading trace data ([caret_analyze #284](https://github.com/tier4/caret_analyze/pull/284))

### v0.4.7 <small>\_ Apr 17, 2023</small> {id = "0.4.7"}

- Refactored the Plot package design ([caret_analyze #267](https://github.com/tier4/caret_analyze/pull/267)), ([caret_analyze #275](https://github.com/tier4/caret_analyze/pull/275)), ([caret_analyze #277](https://github.com/tier4/caret_analyze/pull/277)), ([caret_analyze #278](https://github.com/tier4/caret_analyze/pull/278)), ([caret_analyze #279](https://github.com/tier4/caret_analyze/pull/279)), ([caret_analyze #280](https://github.com/tier4/caret_analyze/pull/280))

### v0.4.6 <small>\_ Apr 3, 2023</small> {id = "0.4.6"}

- **New**: Added function to visualize response time with the stacked bar graph([caret_analyze #265](https://github.com/tier4/caret_analyze/pull/265), [caret_analyze #259](https://github.com/tier4/caret_analyze/pull/259), [caret_doc #169](https://github.com/tier4/caret_doc/pull/169))
- **Update**: Added selective path definition to include or exclude first and last callback execution ([caret_analyze #263](https://github.com/tier4/caret_analyze/pull/263), [caret_analyze #271](https://github.com/tier4/caret_analyze/pull/271), [caret_analyze #272](https://github.com/tier4/caret_analyze/pull/272), [caret_analyze #273](https://github.com/tier4/caret_analyze/pull/273))
- Added a new function to get difference between two Architecture objects ([caret_analyze #245](https://github.com/tier4/caret_analyze/pull/245), [caret_doc #168](https://github.com/tier4/caret_doc/pull/168))
- Removed deprecated interface such as `callback_sched` and `Plot.create_*_plot` ([caret_analyze #266](https://github.com/tier4/caret_analyze/pull/266))
- Refactored visualization API such as `message_flow` ([caret_analyze #267](https://github.com/tier4/caret_analyze/pull/267))

### v0.4.5 <small>\_ Mar 20, 2023</small> {id = "0.4.5"}

- Enhanced the trace filtering to exclude selected DDS events ([caret_trace #101](https://github.com/tier4/caret_trace/pull/101))
- Added the unit to the label of y-axis on time-series graph ([caret_analyze #256](https://github.com/tier4/caret_analyze/pull/256))
- Refactored the Plot package ([caret_analyze #257](https://github.com/tier4/caret_analyze/pull/257), [caret_analyze #258](https://github.com/tier4/caret_analyze/pull/258))
- Added the docstring for explaining rename methods for Architecture object ([caret_analyze #253](https://github.com/tier4/caret_analyze/pull/253))

### v0.4.4 <small>\_ Feb 21, 2023</small> {id = "0.4.4"}

- Fixed the time format of the time-series graphs ([caret_analyze #255](https://github.com/tier4/caret_analyze/pull/255))
- Added the design documentation for trace filtering to exclude events in DDS-layer ([caret_doc #158](https://github.com/tier4/caret_doc/pull/158))
- Moved `get_range()` function to `record` package ([caret_analyze #254](https://github.com/tier4/caret_analyze/pull/254))

### v0.4.3 <small>\_ Feb 10, 2023</small> {id = "0.4.3"}

- Fixed the bug preventing `check_ctf` from running properly after supporting activation/deactivation on runtime ([caret_analyze #251](https://github.com/tier4/caret_analyze/pull/251))
- Suppressed update frequency of progress bar from `tqdm` when reading trace data ([caret_analyze #248](https://github.com/tier4/caret_analyze/pull/248))
- Fixed incorrect annotation from `list` to `List` ([caret_analyze #252](https://github.com/tier4/caret_analyze/pull/252))
- Changed `setup_caret.bash` script to prompt users to install the packages ([caret #72](https://github.com/tier4/caret/pull/72))

### v0.4.2 <small>\_ Jan 23, 2023</small> {id = "0.4.2"}

- Added lightweight recording option to `ros2caret` ([ros2caret #63](https://github.com/tier4/ros2caret/pull/63))
- Removed old interface of `Plot.create_callback_jitter_plot` ([caret_analyze #231](https://github.com/tier4/caret_analyze/pull/231))
- Refactored `Plot.create_callback_scheduling_plot` ([caret_analyze #240](https://github.com/tier4/caret_analyze/pull/240))
- Added new API to combine two paths found by `Architecture.search_paths()` ([caret_analyze #224](https://github.com/tier4/caret_analyze/pull/224))
- Fixed the bug in `Records` class ([caret_analyze #246](https://github.com/tier4/caret_analyze/pull/246))
- Applied minor refactoring ([caret_analyze #244](https://github.com/tier4/caret_analyze/pull/244), [caret_analyze #243](https://github.com/tier4/caret_analyze/pull/243), [caret_analyze #237](https://github.com/tier4/caret_analyze/pull/237), [caret_analyze #234](https://github.com/tier4/caret_analyze/pull/234))
- Suppressed warning message outputted during runtime recording ([caret_analyze #238](https://github.com/tier4/caret_analyze/pull/238), [caret_analyze #236](https://github.com/tier4/caret_analyze/pull/236))
- Added `construction_order` item to identify multiple callbacks which have common symbol ([caret_analyze #225](https://github.com/tier4/caret_analyze/pull/225))

### v0.4.1 <small>\_ Dec 26, 2022</small> {id = "0.4.1"}

- **New**: Added functions to assign intra-node data path with `message_context` ([caret_analyze #196](https://github.com/tier4/caret_analyze/pull/196))
- **Update**: Changed design of visualization API for refactoring to avoid usage of Pandas ([caret_analyze #116](https://github.com/tier4/caret_analyze/pull/116), [caret_analyze #167](https://github.com/tier4/caret_analyze/pull/167), [caret_analyze #168](https://github.com/tier4/caret_analyze/pull/168), [caret_analyze #170](https://github.com/tier4/caret_analyze/pull/170))
- Added configure file to control log level to suppress warning message ([caret_analyze #220](https://github.com/tier4/caret_analyze/pull/220))
- Changed behavior of visualization to output empty graph if inputted data does not have any data rather than raising exception ([caret_analyze #223](https://github.com/tier4/caret_analyze/pull/223))
- Fixed `ValueError`, when `create_publish_subscription_frequency_plot` is called with being inputted empty object ([caret_analyze #227](https://github.com/tier4/caret_analyze/pull/227))
- Added metric name to tooltip for identifying plots on a graph

### v0.4.0 <small>\_ Dec 16, 2022</small> {id = "0.4.0"}

- **New**: Added a new feature to start, stop, and resume even recording whenever users want to ([caret_trace #68](https://github.com/tier4/caret_trace/pull/68), [caret_analyze #190](https://github.com/tier4/caret_analyze/pull/190), [ros2caret #48](https://github.com/tier4/ros2caret/pull/48), and [caret_doc #126](https://github.com/tier4/caret_doc/pull/126))

### v0.3.4 <small>\_ Dec 12, 2022</small> {id = "0.3.4"}

- **New**: Added functions to rename subsystems in Architecture object like executor, node, callback and topic ([caret_analyze#156](https://github.com/tier4/caret_analyze/pull/156))
- Suppressed warning messages, which are outputted because `service` event is not supported, during trace data loading ([caret_analyze#192](https://github.com/tier4/caret_analyze/pull/192))
- Fixed bug which calculated incorrect period or frequency ([caret_analyze#213](https://github.com/tier4/caret_analyze/pull/213))
- Fixed error messages from `Mypy` ([caret_analyze#211](https://github.com/tier4/caret_analyze/pull/211))

### v0.3.3 <small>\_ Nov 28, 2022</small> {id = "0.3.3"}

- Added guidances for a beginner to avoid getting stuck in unexpected cases ([caret_analyze #200](https://github.com/tier4/caret_analyze/pull/200) and [caret_analyze #186](https://github.com/tier4/caret_analyze/pull/186))
- Added `publishers` and `subscriptions` properties to `Architecture` and `Application` classes ([caret_analyze #179](https://github.com/tier4/caret_analyze/pull/179), [caret_analyze #180](https://github.com/tier4/caret_analyze/pull/180))
- Added function to update cache file named as `caret_converted` ([caret_analyze#189](https://github.com/tier4/caret_analyze/pull/189))
- Suppressed unnecessary warnings in `check_caret_rclcpp` ([ros2caret #53](https://github.com/tier4/ros2caret/pull/53), [ros2caret #54](https://github.com/tier4/ros2caret/pull/54))
- Improved documentation of installation ([caret_doc #111](https://github.com/tier4/caret_doc/pull/111)), configuration ([caret_doc #112](https://github.com/tier4/caret_doc/pull/112)), visualization ([caret_doc #98](https://github.com/tier4/caret_doc/pull/98)), and design ([caret_doc#106](https://github.com/tier4/caret_doc/pull/106))
- Fixed a bug which makes incorrect graphs ([caret_analyze #201](https://github.com/tier4/caret_analyze/pull/201))

### v0.3.2 <small>\_ Nov 14, 2022</small> {id = "0.3.2"}

- Improved warning messages of caret_analyze ([caret_analyze #144](https://github.com/tier4/caret_analyze/pull/144), [caret_analyze #158](https://github.com/tier4/caret_analyze/pull/158), [caret_analyze #162](https://github.com/tier4/caret_analyze/pull/162), [caret_analyze #172](https://github.com/tier4/caret_analyze/pull/172), [caret_analyze #182](https://github.com/tier4/caret_analyze/pull/182))
- Added properties of `publishers` and `subscriptions` to `Application` and `Architecture` object([caret_analyze #179](https://github.com/tier4/caret_analyze/pull/179))
- Improved ros2caret([ros2caret #44](https://github.com/tier4/ros2caret/pull/44), [ros2caret #50](https://github.com/tier4/ros2caret/pull/50))
- Enlarged buffer size([ros2_tracing #2](https://github.com/tier4/ros2_tracing/pull/2))
- Added some minor changes ([caret_analyze #183](https://github.com/tier4/caret_analyze/pull/183), [caret_analyze #187](https://github.com/tier4/caret_analyze/pull/187))

### v0.3.1 <small>\_ Oct 31, 2022</small> {id = "0.3.1"}

- Added record function to ros2caret for upcoming feature. ([ros2caret #38](https://github.com/tier4/ros2caret/pull/38))
- Added function to display histogram for response time ([caret_analyze #165](https://github.com/tier4/caret_analyze/pull/165))
- Added API to calculate period and latency ([caret_analyze #167](https://github.com/tier4/caret_analyze/pull/167), [caret_analyze #168](https://github.com/tier4/caret_analyze/pull/168))
- Fixed some bugs ([caret_analyze #141](https://github.com/tier4/caret_analyze/pull/141), [caret_analyze #153](https://github.com/tier4/caret_analyze/pull/153), [caret_analyze #157](https://github.com/tier4/caret_analyze/pull/157))
- Refactored `Architecture` object interface ([caret_analyze](https://github.com/tier4/caret_analyze/pull/84))
- Added some minor changes ([caret_analyze #139](https://github.com/tier4/caret_analyze/pull/139), [caret_analyze #146](https://github.com/tier4/caret_analyze/pull/146), [caret_analyze #147](https://github.com/tier4/caret_analyze/pull/147), [caret_analyze #151](https://github.com/tier4/caret_analyze/pull/151), [caret_analyze #159](https://github.com/tier4/caret_analyze/pull/159), [caret_analyze #162](https://github.com/tier4/caret_analyze/pull/162), [caret_analyze #165](https://github.com/tier4/caret_analyze/pull/165), [caret_analyze#171](https://github.com/tier4/caret_analyze/pull/171))

### v0.3.0 <small>\_ Sept 26, 2022</small> {id = "0.3.0"}

- Supported OS: Ubuntu 22.04
- Supported ROS Dist.: ROS Humble
- Imported to Ubuntu 22.04 and ROS Humble
- Added new feature to show response time ([caret_analyze #96](https://github.com/tier4/caret_analyze/pull/96))
- Added new APIs to visualize frequency, period, and latency of publishes, subscriptions, and communications ([caret_analyze #124](https://github.com/tier4/caret_analyze/pull/124#pullrequestreview-1098527296), [#130](https://github.com/tier4/caret_analyze/pull/130), [#133](https://github.com/tier4/caret_analyze/pull/133), [#134](https://github.com/tier4/caret_analyze/pull/134), [#136](https://github.com/tier4/caret_analyze/pull/136), [#140](https://github.com/tier4/caret_analyze/pull/140))
- Reduced memory consumption wasted for recorded data ([caret_analyze #100](https://github.com/tier4/caret_analyze/pull/100))
- Launched [caret_analyze API document](https://tier4.github.io/caret_analyze/latest/)
- Added small fixes
  - [caret_analyze #107](https://github.com/tier4/caret_analyze/pull/106)
  - [caret_analyze #107](https://github.com/tier4/caret_analyze/pull/107)
  - [caret_analyze #108](https://github.com/tier4/caret_analyze/pull/108)
  - [caret_analyze #109](https://github.com/tier4/caret_analyze/pull/109)
  - [caret_analyze #112](https://github.com/tier4/caret_analyze/pull/112)
  - [caret_analyze #115](https://github.com/tier4/caret_analyze/pull/115)

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
