# performance_test

The performance_test tool tests latency and other performance metrics of [various middleware
implementations](#middleware-plugins) that support a pub/sub pattern. It is used to simulate
non-functional performance of your application.

The performance_test tool allows you to quickly set up a pub/sub configuration,
e.g. number of publisher/subscribers, message size, QOS settings, middleware. The following metrics
are automatically recorded when the application is running:

- **latency**: corresponds to the time a message takes to travel from a publisher to subscriber. The
  latency is measured by timestamping the sample when it's published and subtracting the timestamp
    (from the sample) from the measured time when the sample arrives at the subscriber (only logged when a subscriber is created)
- **CPU usage**: percentage of the total system wide CPU usage (logged separately for each instance of `perf_test`)
- **resident memory**: heap allocations, shared memory segments, stack (used for system's internal
  work) (logged separately for each instance of `perf_test`)
- **sample statistics**: number of samples received, sent, and lost per experiment run.

This `master` branch is compatible with the following ROS2 versions
- rolling
- humble
- galactic
- foxy
- eloquent
- dashing
- Apex.OS

## How to use this document

1. Start [here](#example) for a quick example of building and running the performance_test tool
   with the Cyclone DDS plugin.
2. If needed, find more detailed information about [building](#building-the-performance_test-tool)
   and [running](#running-an-experiment)
3. Or, if the quick example is good enough, skip ahead to the [list of supported middleware
   plugins](#middleware-plugins) to learn how to test a specific middleware implementation.
4. Check out the [tools for visualizing the results](#analyze-the-results)
5. If desired, read about the [design and architecture](#architecture) of the tool.

## Example

This example shows how to test the non-functional performance of the following configuration:

| Option                     | Value       |
|----------------------------|-------------|
| Plugin                     | Cyclone DDS |
| Message type               | Array1k     |
| Publishing rate            | 100Hz       |
| Topic name                 | test_topic  |
| Duration of the experiment | 30s         |
| Number of publisher(s)        | 1 (default) |
| Number of subscriber(s)     | 1 (default) |

1. Install [ROS 2](https://docs.ros.org/en/rolling/index.html)

2. Install [Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds) to /opt/cyclonedds

3. Build performance_test with the [CMake build flag](#eclipse-cyclone-dds) for Cyclone DDS:

    ```bash
    source /opt/ros/rolling/setup.bash
    cd ~/perf_test_ws
    colcon build --cmake-args -DPERFORMANCE_TEST_CYCLONEDDS_ENABLED=ON
    source ./install/setup.bash
    ```

4. Run with the [communication plugin option](#eclipse-cyclone-dds) for Cyclone DDS:

```bash
mkdir experiment
./install/performance_test/lib/performance_test/perf_test --communication CycloneDDS
                                                          --msg Array1k
                                                          --rate 100
                                                          --topic test_topic
                                                          --max-runtime 30
                                                          --logfile experiment/log.csv
```

At the end of the experiment, a CSV log file will be generated in the experiment folder with a name
that starts with `log`.

## Building the performance_test tool

For a simple example, see [Dockerfile.rclcpp](dockerfiles/Dockerfile.rclcpp).

The performance_test tool is structured as a ROS 2 package, so `colcon` is used to build it.
Therefore, you must source a ROS 2 installation:

```bash
source /opt/ros/rolling/setup.bash
```

Select a middleware plugin from [this list](#middleware-plugins).
Then build the performance_test tool with the selected middleware:

```bash
mkdir -p ~/perf_test_ws/src
cd ~/perf_test_ws/src
git clone https://gitlab.com/ApexAI/performance_test.git
cd ..
# At this stage, you need to choose which middleware you want to use
# The list of available flags is described in the middleware plugins section
# Square brackets denote optional arguments, like in the Python documentation.
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release <cmake_enable_plugin_flag>
source install/setup.bash
```

## Running an experiment

The performance_test experiments are run through the `perf_test` executable.
To find the available settings, run with `--help` (note the required and default arguments):

```bash
~/perf_test_ws$ ./install/performance_test/lib/performance_test/perf_test --help
```

- The `-c` argument should match the selected [middleware plugin](#middleware-plugins)
  from the build phase.
- The `--msg` argument should be one of the supported message types, which can be listed
  by running with `--msg-list`.

### Single machine or distributed system?

Based on the configuration you want to test, the usage of the performance_test tool differs. The
different possibilities are explained below.

For running tests on a single machine, you can choose between the following options:

1. Intraprocess means that the publisher and subscriber threads are in the same process.

    ```bash
    perf_test <options> --num-sub-threads 1 --num-pub-threads 1
    ```

1. Interprocess means that the publisher and subscriber are in different processes. To test
   interprocess communication, two instances of the performance_test must be run, e.g.

    ```bash
    # Start the subscriber first
    perf_test <options> --num-sub-threads 1 --num-pub-threads 0 &
    sleep 1  # give the subscriber time to finish initializing
    perf_test <options> --num-sub-threads 0 --num-pub-threads 1
    ```

On a distributed system, testing latency is difficult, because the clocks are probably not
perfectly synchronized between the two devices. To work around this, the performance_test tool
supports relay mode, which allows for a round-trip style of communication:

```bash
# On the main machine
perf_test <options> --roundtrip-mode Main

# On the relay machine:
perf_test <options> --roundtrip-mode Relay
```

In relay mode, the Main machine sends messages to the Relay machine, which immediately sends the
messages back. The Main machine receives the relayed message, and reports the round-trip latency.
Therefore, the reported latency will be roughly double the latency compared to the latency reported
in non-relay mode.

## Middleware plugins

The performance test tool can measure the performance of a variety of communication solutions
from different vendors. In this case there is no [rclcpp or rmw
layer](http://docs.ros2.org/beta2/developer_overview.html#internal-api-architecture-overview)
overhead over the publisher and subscriber routines.

The performance_test tool implements an executor that runs the publisher(s) and/or the subscriber(s) in
their own thread.

 The following plugins are currently implemented:

### Eclipse Cyclone DDS

- [Eclipse Cyclone DDS 0.9.0b1](https://github.com/eclipse-cyclonedds/cyclonedds/tree/0.9.0b1)
- CMake build flag: `-DPERFORMANCE_TEST_CYCLONEDDS_ENABLED=ON`
- Communication plugin: `-c CycloneDDS`
- Docker file: [Dockerfile.CycloneDDS](dockerfiles/Dockerfile.CycloneDDS)
- Available transports:
  - Cyclone DDS zero copy requires the
    [runtime switch](https://github.com/eclipse-cyclonedds/cyclonedds/blob/iceoryx/docs/manual/shared_memory.rst)
    to be enabled.
  - When the runtime switch is enabled,
    [RouDi](https://github.com/eclipse-iceoryx/iceoryx/blob/master/doc/website/getting-started/overview.md#roudi)
    must be running.
  - If the runtime switch is enabled, but `--zero-copy` is not added, then the plugin will not use
    the loaned sample API, but iceoryx will still transport the samples.
  - See [Dockerfile.mashup](dockerfiles/Dockerfile.mashup)
  -
    | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
    |-------|---------------------|--------------------|
    | INTRA (default), LoanedSamples (`--zero-copy`) | UDP (default), LoanedSamples (`--zero-copy`), SHMEM (enable runtime switch)                 | UDP                |

### Eclipse Cyclone DDS C++ binding

- [Eclipse Cyclone DDS C++ bindings 0.9.0b1](https://github.com/eclipse-cyclonedds/cyclonedds-cxx/tree/0.9.0b1)
- CMake build flag: `-DPERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED=ON`
- Communication plugin: `-c CycloneDDS-CXX`
- Docker file: [Dockerfile.CycloneDDS-CXX](dockerfiles/Dockerfile.CycloneDDS-CXX)
- Available transports:
  - Cyclone DDS zero copy requires the
    [runtime switch](https://github.com/eclipse-cyclonedds/cyclonedds/blob/iceoryx/docs/manual/shared_memory.rst)
    to be enabled.
  - When the runtime switch is enabled,
    [RouDi](https://github.com/eclipse-iceoryx/iceoryx/blob/master/doc/website/getting-started/overview.md#roudi)
    must be running.
  - If the runtime switch is enabled, but `--zero-copy` is not added, then the plugin will not use
    the loaned sample API, but iceoryx will still transport the samples.
  - See [Dockerfile.mashup](dockerfiles/Dockerfile.mashup)
  -
    | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
    |-------|---------------------|--------------------|
    | INTRA (default), LoanedSamples (`--zero-copy`), SHMEM (enable runtime switch) | UDP (default), LoanedSamples (`--zero-copy`), SHMEM (enable runtime switch)                 | UDP                |

### Eclipse iceoryx

- [iceoryx 1.0](https://github.com/eclipse-iceoryx/iceoryx/tree/release_1.0)
- CMake build flag: `-DPERFORMANCE_TEST_ICEORYX_ENABLED=ON`
- Communication plugin: `-c iceoryx`
- Docker file: [Dockerfile.iceoryx](dockerfiles/Dockerfile.iceoryx)
- The iceoryx plugin is not a DDS implementation.
  - The DDS-specific options (such as domain ID, durability, and reliability) do not apply.
- To run with the iceoryx plugin,
  [RouDi](https://github.com/eclipse-iceoryx/iceoryx/blob/master/doc/website/getting-started/overview.md#roudi)
  must be running.
- Available transports:
  | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
  |-----------|---------------------|-----------------------------------|
  | SHMEM (default), LoanedSamples (`--zero-copy`) | SHMEM (default), LoanedSamples (`--zero-copy`) | Not supported by performance_test |

### eProsima Fast DDS

- [FastDDS 2.6.0](https://github.com/eProsima/Fast-DDS/tree/v2.6.0)
- CMake build flag: `-DPERFORMANCE_TEST_FASTRTPS_ENABLED=ON`
- Communication plugin: `-c FastRTPS`
- Docker file: [Dockerfile.FastDDS](dockerfiles/Dockerfile.FastDDS)
- Available transports:
  | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
  |-------|---------------------|--------------------|
  | UDP   | UDP                 | UDP                |

### OCI OpenDDS

- [OpenDDS 3.13.2](https://github.com/objectcomputing/OpenDDS/tree/DDS-3.13.2)
- CMake build flag: `-DPERFORMANCE_TEST_OPENDDS_ENABLED=ON`
- Communication plugin: `-c OpenDDS`
- Docker file: [Dockerfile.OpenDDS](dockerfiles/Dockerfile.OpenDDS)
- Available transports:
  | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
  |-------|---------------------|--------------------|
  | TCP   | TCP                 | TCP                |

### RTI Connext DDS

- [RTI Connext DDS 5.3.1+](https://www.rti.com/products/connext-dds-professional)
- CMake build flag: `-DPERFORMANCE_TEST_CONNEXTDDS_ENABLED=ON`
- Communication plugin: `-c ConnextDDS`
- Docker file: Not available
- A license is required
- You need to source an RTI Connext DDS environment.
  - If RTI Connext DDS was installed with ROS 2 (Linux only):
    - `source /opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash`
  - If RTI Connext DDS is installed separately, you can source the following script to set the
    environment:
    - `source <connextdds_install_path>/resource/scripts/rtisetenv_<arch>.bash`
- Available transports:
  | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
  |-------|---------------------|--------------------|
  | INTRA | SHMEM               | UDP                |

### RTI Connext DDS Micro

- [Connext DDS Micro 3.0.3](https://www.rti.com/products/connext-dds-micro)
- CMake build flag: `-DPERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED=ON`
- Communication plugin: `-c ConnextDDSMicro`
- Docker file: Not available
- A license is required
- Available transports:
  | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
  |-------|---------------------|--------------------|
  | INTRA | SHMEM               | UDP                |

## Framework plugins

The performance_test tool can also measure the end-to-end latency of a framework. In this case, the
executor of the framework is used to run the publisher(s) and/or the subscriber(s). The potential overhead of the [rclcpp or rmw
layer](http://docs.ros2.org/beta2/developer_overview.html#internal-api-architecture-overview) is measured.

### ROS 2

The performance test tool can also measure the performance of a variety of RMW implementations,
through the ROS2 `rclcpp::publisher` and `rclcpp::subscriber` API.

- [ROS 2 `rclcpp::publisher` and `rclcpp::subscriber`](https://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- CMake build flag: `-DPERFORMANCE_TEST_RCLCPP_ENABLED=ON` (on by default)
- Communication plugin:
  - Callback with Single Threaded Executor: `-c rclcpp-single-threaded-executor`
  - Callback with Static Single Threaded Executor: `-c rclcpp-static-single-threaded-executor`
  - [`rclcpp::WaitSet`](https://github.com/ros2/rclcpp/pull/1047): `-c rclcpp-waitset`
- Docker file: [Dockerfile.rclcpp](dockerfiles/Dockerfile.rclcpp)
- [Available underlying RMW implementations](https://docs.ros.org/en/rolling/Concepts/About-Different-Middleware-Vendors.html):
  - ROS 2 Rolling is pre-configured to use `rmw_fastrtps_cpp`
  - Follow [these instructions](https://docs.ros.org/en/rolling/Guides/Working-with-multiple-RMW-implementations.html)
      to use a different RMW implementation
- Available transports: depends on underlying RMW implementation
  - LoanedSamples are available (`--zero-copy`) for `ROS_DISTRO = foxy` and above

### Apex.OS

- Apex.OS
- CMake build flag: `-DPERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED=ON`
- Communication plugin: `-c ApexOSPollingSubscription`
- Docker file: Not available
- Available underlying RMW implementations: `rmw_apex_middleware`
- Available transports:
  | Pub/sub in same process | Pub/sub in different processes on same machine | Pub/sub in different machines |
  |-------|---------------------|--------------------|
  | UDP (default), LoanedSamples (`--zero_copy`), SHMEM (configurable with Apex.OS)  | UDP (default), LoanedSamples (`--zero_copy`), SHMEM (configurable with Apex.OS)                | UDP                |

## Analyze the results

After an experiment is run with the `-l` flag, a CSV file is recorded. It is possible to add custom
data to the CSV file by setting the`APEX_PERFORMANCE_TEST` environment variable before running an
experiment, e.g.

```json
# JSON format
export APEX_PERFORMANCE_TEST="
{
\"My Version\": \"1.0.4\",
\"My Image Version\": \"5.2\",
\"My OS Version\": \"Ubuntu 16.04\"
}
"
```

### Plot results

The performance_test tool provides several tools to plot the generated results:

1. Results rendered on a PDF file: handy to share results
    <img src="plotter_generated_pdf.png"  width="1000">
1. Results rendered in a Jupyter notebook: used to compare multiple experiments
    <img src="performance_test/helper_scripts/apex_performance_plotter/example_plot_two_experiments.png"  width="1000">

The plot tool requires python3 and texlive. On an Ubuntu system you will need to
install the following packages:

`sudo apt-get install python3 python3-pip texlive texlive-pictures texlive-luatex texlive-latex-extra`

Start a Python virtual environment and install the required Python packages:

```bash
cd performance_test/helper_scripts/apex_performance_plotter
pip3 install .
```

#### Usage

To generate a PDF from the log file, invoke the `perfplot` binary installed in the previous step:

`perfplot <filename1> <filename2> ...`

Be sure to also check `perfplot -h` for additional options.

>>>
:point_up: **Common Pitfalls**

All of the latency metrics are collected and calculated by the subscriber process.
For interprocess communication, it is recommended to provide different prefixes for
the log files:

```bash
perf_test -c rclcpp-single-threaded-executor --msg Array1k -p 0 -s 1 -l log_sub.csv
perf_test -c rclcpp-single-threaded-executor --msg Array1k -p 1 -s 0 -l log_pub.csv
```

Then, to plot the latency metrics, invoke perfplot on the subscriber's log file.
If perfplot is invoked on the publisher's log file, then the CPU and memory
metrics will be plotted, but the latency plot will be empty.
>>>

To analyze the results in a Jupyter notebook run the following commands:

```bash
pipenv shell
jupyter notebook plot_logs.ipynb

# When you are done, deactivate the venv
deactivate
```

## Architecture

Apex.AI's _Performance Testing in ROS 2_ white paper
([available here](https://drive.google.com/file/d/15nX80RK6aS8abZvQAOnMNUEgh7px9V5S/view))
describes how to design a fair and unbiased performance test, and is the basis for this project.
<center><img src="architecture.png"></center>

## Future extensions and limitations

- Communication frameworks like DDS have a huge amount of settings. This tool only allows the most
  common QOS settings to be configured. The other QOS settings are hardcoded in the application.
- Only one publisher per topic is allowed, because the data verification logic does not support
  matching data to the different publishers.
- Some communication plugins can get stuck in their internal loops if too much data is received.
  Figuring out ways around such issues is one of the goals of this tool.
- FastRTPS wait-set does not support timeouts which can lead to the receiving not aborting. In that
  case the performance test must be manually killed.
- Using Connext DDS Micro INTRA transport with `reliable` QoS and history kind set to `keep_all`
  [is not supported with Connext
  Micro](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/html/usersmanual/transports/INTRA.html#reliability-and-durability).
  Set `keep-last` as QoS history kind always when using `reliable`.

Possible additional communication which could be implemented are:

- Raw UDP communication

## Building with limited resources

When building this tool, the compiler must perform a lot of template expansion. This can be
overwhelming for a system with a low-power CPU or limited RAM. There are some additional CMake
options which can reduce the system load during compilation:

1. The [ROS 2 Middleware plugins](#ros-2-middleware-plugins) are enabled by default. If only a
[native plugin](#native-plugins) is needed, then the ROS 2 plugins can be disabled by adding
`-DPERFORMANCE_TEST_RCLCPP_ENABLED=OFF` to the `--cmake-args`.
1. This tool includes many different message types, each with many different sizes. Reduce the number of
messages, and thus the compilation load, by adding `-DPERFORMANCE_TEST_MINIMAL_MESSAGES` to the
`--cmake-args`. The reduced message set can be found [here](performance_test/msg/CMakeLists.txt#L73-86).
