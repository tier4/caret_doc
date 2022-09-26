# Changelog

## CARET

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
