# Changelog for package performance_test

## X.Y.Z (YYYY/MM/DD)

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
