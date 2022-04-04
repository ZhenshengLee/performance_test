# performance_report

This package serves two purposes:

1. Run multiple `performance_test` experiments
2. Plot the combined results of those experiments

## Quick start

Install the required dependencies

```
# install bokeh and selenium
python3 -m pip install bokeh selenium pandas
# install firefox-geckodriver
sudo apt install firefox-geckodriver
```

Note: all the commands below are ran from the `colcon_ws` where `performance_test/performance_report` is installed.

```
# Build performance_test and performance_report
colcon build

# Set up the environment
source install/setup.bash

# Run perf_test for each experiment in the yaml file
ros2 run performance_report runner \
  --log-dir perf_logs \
  --test-name experiments \
  --configs src/performance_test/performance_report/cfg/runner/run_one_experiment.yaml

# The runner generates log files to the specified directory: `./perf_logs/experiements/`

# Generate the plots configured in the specified yaml file
ros2 run performance_report plotter \
  --log-dir perf_logs \
  --configs src/performance_test/performance_report/cfg/plotter/plot_one_experiment.yaml

# The generated plots will be saved in `./perf_logs`

# Generate the reports configured in the specified yaml file
ros2 run performance_report reporter \
  --log-dir perf_logs \
  --configs src/performance_test/performance_report/cfg/reporter/report_one_experiment.yaml
```

The above example only runs and plots one experiement, however multiple experiements can be ran
automatically by specifying it in the configuration file.

Try the above commands again, this time replacing the `yaml` file with `*_many_experiements.yaml`
for each executable.

When running many experiments the `runner` will by default skip any experiments that already have
log files generated in the specified `log_dir`. This can be overridden by adding the `-f` or
`--force` argument to the command.

## Running the same experiments on multiple platforms

Suppose you want to run an experiment on multiple platforms, then combine the results into a single
report. First, pass the `--test-name` arg to `runner`, to differentiate the result sets:

```
# on platform 1:
ros2 run performance_report runner --test_name platform1 -l log_dir -c run.yaml
# results will be stored in ./log_dir/platform1/

# on platform 2:
ros2 run performance_report runner --test_name platform2 -l log_dir -c run.yaml
# results will be stored in ./log_dir/platform2/
```

You can then combine these results into a single `log_dir`, on the platform where you will run
`plotter` or `reporter`. Then, in your `plotter` or `reporter` configuration file, set `test_name`
in each dataset, to select results from that platform's result set:

```yaml
# report.yaml
datasets:
  dataset_p1:
    test_name: platform1  # this matches the --test-name passed to runner
    # other fields...
  dataset_p2:
    test_name: platform2  # this matches the --test-name passed to runner
    # other fields...
reports:
  # ...
```

```
ros2 run performance_report reporter -l log_dir -c report.yaml
```

## Notes

- Currently, this tool is intended for ROS 2 with rmw_cyclone_dds, or Apex.OS with
  Apex.Middleware. It has not been tested with any other transport.
- If the run configuration includes `SHMEM` or `ZERO_COPY` transport, then a file for
  configuring the middleware will be created to enable the shared memory transfer.
  - You must start RouDi before running the experiments. This tool will not automatically
    start it for you.
