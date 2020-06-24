# Introduction

**Default Version Support:** ROS2 Dashing, Fast-DDS 2.0.x

This performance test tool allows you to test performance and latency of various communication means
like ROS 2, ROS 2 Waitset, FastDDS, Connext DDS Micro, Eclipse Cyclone DDS and OpenDDS.

It can be extended to other communication frameworks easily.

A detailed description can be found here: [Design Article](performance_test/design/performance_test-design.md)

# Building and running performance test

## Installing dependencies

ROS 2: https://github.com/ros2/ros2/wiki/Installation

Additional dependencies are Java and others declared in the `package.xml` file
```
sudo apt-get install default-jre
rosdep install -y --from performance_test --ignore-src
```

## How to build

```
source <ros2_install_path>/setup.bash
mkdir -p perf_test_ws/src
cd perf_test_ws/src
git clone https://gitlab.com/ApexAI/performance_test.git
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## How to run a single experiment

After building, a simple experiment can be run using the following.

Before you start create a directory for the output.

```
mkdir experiment
cd experiment
./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t Array1k --max_runtime 30
```
At the end of the experiment, a CSV log file will be generated in the experiment folder (e.g. `experiment/log_Array1k_<current_date>`

# Generating graphical plots

## Plot results

To plot the results, you will need to install the perfplot tool from the apex_performance_plotter python module.
See [apex_performance_plotter](https://gitlab.com/ApexAI/performance_test/-/tree/master/performance_test/helper_scripts/apex_performance_plotter)
for the list of dependecies.

```
pip3 install performance_test/helper_scripts/apex_performance_plotter
```

This tool will convert performance test log files into PDFs containing graphs of the results.

Note: Some of the dependencies of apex_performance_plotter (specifically pandas, at the time of writing) require python 3.6. It is possible to get apex_performance_plotter working with older dependencies that run with python 3.5, but that is beyond the scope of this document.

In order to have the log-file plotted into a PDF file, specify the log file name after the plotter's tool executable:
```
perfplot <logfile_name>
```

This will generate a PDF file `<logfile_name>.pdf` that can be viewed in any PDF viewer.

# Configuration options provided by the tool

The tool has a fully documented command line interface which can be accessed by typing
`performance_test --help`.

```
~/perf_test_ws$ ./install/performance_test/lib/performance_test/perf_test --help

Allowed options:
  -h [ --help ]                        Print usage message.
  -l [ --logfile ] arg                 Optionally specify a logfile.
  -r [ --rate ] arg (=1000)            The rate data should be published.
                                       Defaults to 1000 Hz. 0 means publish as
                                       fast as possible.
  -c [ --communication ] arg           Communication plugin to use (ROS2,
                                       FastRTPS, ConnextDDSMicro, CycloneDDS,
                                       OpenDDS, ROS2PollingSubscription)
  -t [ --topic ] arg                   Topic to use. Use --topic_list to get a
                                       list.
  --topic_list                         Prints list of available topics and
                                       exits.
  --dds_domain_id arg (=0)             Sets the DDS domain id.
  --reliable                           Enable reliable QOS. Default is best
                                       effort.
  --transient                          Enable transient QOS. Default is
                                       volatile.
  --keep_last                          Enable keep last QOS. Default is keep
                                       all.
  --history_depth arg (=1000)          Set history depth QOS. Defaults to 1000.
  --disable_async                      Disables async. pub/sub.
  --max_runtime arg (=0)               Maximum number of seconds to run before
                                       exiting. Default (0) is to run forever.
  -p [ --num_pub_threads ] arg (=1)    Maximum number of publisher threads.
  -s [ --num_sub_threads ] arg (=1)    Maximum number of subscriber threads.
  --use_ros_shm                        Use Ros SHM support.
  --check_memory                       Prints backtrace of all memory
                                       operations performed by the middleware.
                                       This will slow down the application!
  --use_rt_prio arg (=0)               Set RT priority. Only certain platforms
                                       (i.e. Drive PX) have the right
                                       configuration to support this.
  --use_rt_cpus arg (=0)               Set RT cpu affinity mask. Only certain
                                       platforms (i.e. Drive PX) have the right
                                       configuration to support this.
  --use_drive_px_rt                    alias for --use_rt_prio 5 --use_rt_cpus
                                       62
  --use_single_participant             Uses only one participant per process.
                                       By default every thread has its own.
  --no_waitset                         Disables the wait set for new data. The
                                       subscriber takes as fast as possible.
  --no_micro_intra                     Disables the Connext DDS Micro INTRA
                                       transport.
  --with_security                      Make nodes with deterministic names for
                                       use with security
  --roundtrip_mode arg (=None)         Selects the round trip mode (None, Main,
                                       Relay).
  --ignore arg (=0)                    Ignores first n seconds of the
                                       experiment.
  --disable_logging                    Disables experiment logging to stdout.
  --expected_num_pubs arg (=0)         Expected number of publishers for
                                       wait_for_matched
  --expected_num_subs arg (=0)         Expected number of subscribers for
                                       wait_for_matched
  --wait_for_matched_timeout arg (=30) Maximum time[s] to wait for matching
                                       publishers/subscribers. Defaults to 30s
```

Some things to note:

1. `--no_micro_intra` option is obsolete and will not work with the Connext Micro plugin.
2. `--use_single_participant` option also should not be used as its obsolete and will be removed soon.
3. `--use_ros_shm` enables [ROS 2 INTRA transport](https://index.ros.org/doc/ros2/Tutorials/Intra-Process-Communication/).

## Implemented plugins

The performance test tool can measure the performance of a variety of communication middlewares from different vendors. In this case there is no [rclcpp or rmw layer](http://docs.ros2.org/beta2/developer_overview.html#internal-api-architecture-overview) overhead over the publisher and subscriber routines. The following plugins are currently implemented:

| RAW DDS Plugin                                                                                                                    | Supported subscription | Supported transports | `--cmake-args` to pass when building performance_test    | Communication mean (-c) to pass when running experiments |
|-----------------------------------------------------------------------------------------------------------------------------------|------------------------|----------------------|----------------------------------------------------------|----------------------------------------------------------|
| [FastDDS 2.0.x](https://github.com/eProsima/Fast-RTPS/tree/2.0.x)                                                                 | Native DDS Code        | UDP                  | `-DPERFORMANCE_TEST_FASTRTPS_ENABLED=ON`                 | FastRTPS                                                 |
| [Connext DDS Micro 3.0.2](https://www.rti.com/products/connext-dds-micro) (will only work if Apex.OS is present)                                                 | Native DDS Code        | INTRA,SHMEM          | `-DPERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED=ON` | ConnextDDSMicro                                      |
| [Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds/tree/4e805597631ed0dcbdc0eecfe9d532cb75180ae7) | Native DDS Code        | UDP                  | `-DPERFORMANCE_TEST_CYCLONEDDS_ENABLED=ON`      | CycloneDDS                                           |
| [OpenDDS 3.13.2](https://github.com/objectcomputing/OpenDDS/tree/DDS-3.13.2)                                          | Native DDS Code        | UDP                  | `-DPERFORMANCE_TEST_OPENDDS_ENABLED=ON`        | OpenDDS                                              |

If you want to use any of these supported plugins, please refer to the table above for the CMAKE arguments to provide while building the tool and specify the appropriate Communication Mean (-c option) when running the experiment.

For example, to run a performance test with the ConnextMicro plugin, build performance_test with the following command:
```
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DPERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED=ON
```
Now to run the performance test with ConnextDDSMicro plugin :
```
./install/performance_test/lib/performance_test/perf_test -c ConnextDDSMicro -l log -t Array1k --max_runtime 10
```

## Supported rmw implementations

The performance_test tool can also measure performance of the application with the [ROS 2 layers](http://docs.ros2.org/beta2/developer_overview.html#internal-api-architecture-overview). For example the following configuration can be tested: `RTI Connext Micro 3.0.2 + ROS2PollingSubscription rclcpp + rmw_apex_dds`. Performance_test tool supports [`ROS 2 Dashing`](https://index.ros.org/doc/ros2/Installation/Dashing/) version.
The following plugins with a ROS middleware interface are currently supported:

| RMW Implementation                                                                       | Supported subscription                                 | Supported transports |  `--cmake-args` to pass when building performance_test                                                                                  | Communication mean (-c) to pass when running experiments                         |
|------------------------------------------------------------------------------------------|--------------------------------------------------------|----------------------|-------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------|
| [rmw_fastrtps_cpp](https://github.com/ros2/rmw_fastrtps)                                                                         | ROS 2 Callback(enabled by default),<br>Apex.OS WaitSet | UDP                  | Nothing for ROS 2 Callback<br><br>`-DPERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED=ON`<br>(for using Apex.OS waitsets)         | ROS2 (enabled by default)<br><br>ROS2PollingSubscription (for using Apex.OS waitsets) |
| rmw_apex_dds ([Apex.AI](https://www.apex.ai/apex-os) proprietary <br>rmw implementation) | ROS 2 Callback(enabled by default),<br>Apex.OS WaitSet | INTRA,SHMEM          | Nothing for ROS 2 Callback<br><br>`-DPERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED=ON`<br>(for using Apex.OS waitsets) | ROS2 (enabled by default)<br><br>ROS2PollingSubscription (for using Apex.OS waitsets) |
| rmw_cyclonedds_cpp                                                                       | ROS 2 Callback(enabled by default)                     | UDP                  | Nothing for ROS 2 Callback                                                                               | ROS2 (enabled by default)|

Apart from the default ROS 2, you can use the Apex.OS WaitSets by building and running the tool with `ROS2PollingSubscription` as:

```
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DPERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED=ON
```
Now run the experiments as follows:
```
./install/performance_test/lib/performance_test/perf_test -c ROS2PollingSubscription -l log -t Array1k --max_runtime 10
```

> Note:
> - The DDS implementation that Apex.OS has been compiled with (`rmw_fastrtps_cpp` or `rmw_apex_dds`) is automatically linked when the performance_test tool is built with Apex.OS.
> - The ROS2PollingSubscription option only works if Apex.OS is present.
> - The ROS2PollingSubscription option is not yet available with `rmw_cyclonedds_cpp`.

# Batch run experiments (for advanced users)

Multiple experiments can be run using this python script:

```
python3 src/performance_test/performance_test/helper_scripts/run_experiment.py
```

You need to edit the python script to call the performance test tool with the desired configurations.

# Running experiments intraprocess vs running experiments interprocess

The tool offers to run the experiments either in Intraprocess composition which means the publisher and subscriber threads are in the same process or Inter process composition which requires the publisher and subscriber to be in different processes. This is very useful if you want to test the performance of different transports like Micro INTRA, UDP and SHMEM.

Let's take an example of a single publisher and single subscriber:
```
./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t Array1k --max_runtime 30 --num_sub_threads 1 --num_pub_threads 1
```
which is same as running by default:
```
./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t Array1k --max_runtime 30
```
This is example of running the experiments in Intraprocess composition. Connext Micro as per Apex.OS, is configured to use `Micro INTRA` in this setting. FastDDS and other supported DDS implementations use `UDP` by default.

To run the experiments in different processes, the subscriber and publisher processes we can run the tool twice simultaneously. Run the first instance of the tool like :
```
./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t Array1k --max_runtime 30 --num_sub_threads 0 --num_pub_threads 1
```
This is the publisher process. Now to run the subscriber open a second window in the terminal and run a second instance of the tool like:

```
./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t Array1k --max_runtime 30 --num_sub_threads 1 --num_pub_threads 0
```
This is the subscriber process. The tool supports multiple subscribers to be run at once. So you can configure the value of `--num_sub_threads` in the subscriber process to be more than one also.

This is an example of running the experiments in Interprocess composition. Connext Micro as per Apex.OS, is configured to use `SHMEM` in this setting. FastDDS and other supported DDS implementations use `UDP` by default.

> Note:
In Inter process composition the CPU and Resident Memory measurements are logged separately for the publisher and subscriber processes.

# Relay mode

Testing latency between multiple machines is difficult as it is hard precisely synchronize clocks between them.
To overcome this issue performance test supports relay mode which allows for a round-trip style of communication.

On the main machine: `./install/performance_test/lib/performance_test/perf_test -c ROS2 -t Array1k --roundtrip_mode Main`
On the relay machine: `./install/performance_test/lib/performance_test/perf_test -c ROS2 -t Array1k --roundtrip_mode Relay`

Note: On the main machine the round trip latency is reported and will be roughly double the latency compared to the latency reported in non-relay mode.

# Save results to a SQL database

The tool also gives you the ability to persist the performance test results in a SQL compatible database.

See [Add SQL support readme](add_sql_support_readme.md) for instructions and implementation details.

# Memory analysis

You can use OSRF memory tools to find memory allocations in your application. To enable it
you need to do the following steps, assuming you already did compile performance test before:

1. Enter your work space: `cd perf_test_ws/src`
1. Clone OSRF memory memory tools: `git clone https://github.com/osrf/osrf_testing_tools_cpp.git`
1. Build everything `cd .. && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
1. You need to preload the memory library to make diagnostics work: `export LD_PRELOAD=$(pwd)/install/osrf_testing_tools_cpp/lib/libmemory_tools_interpose.so`
1. Run with memory check enabled: `./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t Array1k --max_runtime 10 --check_memory`

> Note: Enabling this feature will cause a huge performance impact.

# Custom environment data

You can set the `APEX_PERFORMANCE_TEST` environment variable before running performance test
to add custom data to the output CSV file.
This information will then also be visible in the files outputted by the plotter script.
Please use the JSON format to pass the values.

Example:
```
export APEX_PERFORMANCE_TEST="
{
\"My Version\": \"1.0.4\",
\"My Image Version\": \"5.2\",
\"My OS Version\": \"Ubuntu 16.04\"
}
"
./install/performance_test/lib/performance_test/perf_test -c ROS2 -t Array1k
```

# Troubleshooting

1. When running performance test it prints for example the following error :
`ERROR: You must compile with FastRTPS support to enable FastDDS as communication mean.`

This means that the performance test needs to be compiled with `--cmake-args -DPERFORMANCE_TEST_FASTRTPS_ENABLED=ON` to switch from ROS 2 to FastDDS.

# Literature

We have attempted to write a white paper with the goal of explaining how to do a fair and unbiased performance testing based on the performance testing framework that we built at Apex.AI and the experience that we gathered in the past 1.5 years. Here is a [link](https://drive.google.com/file/d/15nX80RK6aS8abZvQAOnMNUEgh7px9V5S/view) to the paper.
