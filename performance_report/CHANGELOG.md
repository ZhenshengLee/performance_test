# Changelog for package performance_report

## 1.1.1 (2022/06/07)

### Fixed
- Bokeh line style can be specified in the plotter and reporter .yaml files

## 1.1.0 (2022/06/02)

### Fixed
- Fix the GBP builds by removing `python3-bokeh-pip` from package.xml

## 1.0.0 (2022/05/12)

### Added
- Shared memory experiments are now compatible with both Apex.Middleware and rmw_cyclonedds_cpp
- `commander` tool to emit the commands for running the experiments, instead of running them directly
### Changed
- Use the new perf_test CLI args for QOS settings instead of old flags
### Deprecated
### Removed
### Fixed