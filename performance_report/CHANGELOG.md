# Changelog for package performance_report

## X.Y.Z (YYYY/MM/DD)

## 1.2.1 (2022/06/30)

## 1.2.0 (2022/06/28)

### Changed
- In the `reporter` configuration, the `template_name` value may be an array

## 1.1.2 (2022/06/08)

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
