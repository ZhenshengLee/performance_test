# Changelog for package performance_test

## X.Y.Z (YYYY/MM/DD)

## 1.2.1 (2022/06/30)

### Fixed
- Capture the timestamp as soon as a message is received, instead of just before
  storing the metrics, to reduce the reported latency to a more correct value

## 1.2.0 (2022/06/28)

### Changed
- The CLI arguments for specifying the output type have changed:
   - For console output, updated every second, add `--print-to-console`
   - For file output, use `--logfile my_file.csv` or `--logfile my_file.json`
      - The type will be deduced from the file name
   - If neither of these options is specified, then a warning will print,
     and the experiment will still run
- The linter configurations are now configured locally. This means that the output
  of `colcon test` should be the same no matter the installed ROS distribution.
- The `--zero-copy` arg is now valid even if the publisher and subscriber(s)
  are in the same process
### Removed
- The publisher and subscriber loop reserve metrics are no longer recorded or reported
### Fixed
- CPU usage will no longer be stuck at `0`

### Removed
- The pub/sub loop reserve time metrics

## 1.1.2 (2022/06/08)

### Changed
- Use `steady_clock` for all platforms, including QNX QOS

## 1.1.1 (2022/06/07)

### Changed
- Significant refactor to simplify the analysis pipeline
### Fixed
- Add some missing definitions when Apex.OS is enabled, but the rclcpp plugins are disabled

## 1.1.0 (2022/06/02)

### Added
- New Apex.OS Polling Subscription plugin
- Compatibility with ROS2 Humble

## 1.0.0 (2022/05/12)

### Added
- More expressive perf_test CLI args for QOS settings
- A plugin for [Cyclone DDS with C++ bindings v0.9.0b1](https://github.com/eclipse-cyclonedds/cyclonedds-cxx/tree/0.9.0b1)
### Changed
- CLI args for QOS settings:
    - `--reliability <RELIABLE|BEST_EFFORT>`
    - `--durability <TRANSIENT_LOCAL|VOLATILE>`
    - `--history <KEEP_LAST|KEEP_ALL>`
- `master` branch is compatible with many ROS2 distributions:
    - dashing
    - eloquent
    - foxy
    - galactic
    - rolling
### Deprecated
- CLI flags for QOS settings:
    - `--reliable`
    - `--transient`
    - `--keep-last`
### Removed
- The branches for specific ROS2 distributions have been deleted
### Fixed
- CI jobs and Dockerfiles are decoupled from the middleware bundled with the ROS2 distribution
