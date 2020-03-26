## Performance Test Tool

### Purpose / Use Cases (Why is such a tool required?)

The performance test tool was designed to evaluate the performance of various means of communication
which either support the publish subscribe pattern directly or can be adapted to it.

### Design (How does it work?)

The performance test tool runs at least one publisher and at least one subscriber, each one in one independent thread or process and records different performance metrics.
Each metric value is recorded for every second of the experiment duration. For example if the total experiment runtime is 30 sec, all the metrics logged will have 30 values.

### Measured metrics

#### Latency

The latency corresponds to the time a message takes to travel from a publisher to subscriber. The latency is measured by:

1. Timestamping the sample when it's published.
2. Subtracting the timestamp (from the sample) to the measured time when the sample arrived to the subscriber.

#### CPU Usage

It also measures the CPU usage of the performance test process as a percentage of the total systemwide CPU time.

#### Resident Memory

The performance test tool calculates the resident memory usage of each experiment. The resident memory value mostly comes from:

1. Heap allocations done by DDS - mostly by DataReaders/Writers
2. Shared memory segments (if using SHMEM transport, none otherwise) - can be large depending on the receive buffer size (queue size) and message sizes (payload)
3. Stack (used for system's internal work) - minimal allocation size compared to two previous points

The rough approximation of the resident memory consumption of an experiment can be done with the following formula: `message_size*(history_depth*2)` for every Apex.OS Subscription/DataReader. For example, an experiment publishing messages of type `Array` and size 2MB with history depth of 100 will incur approximately 2MB*(100*2) = 400MB allocation per subscriber thread. Note that this calculation excludes the shared memory segment allocation if SHMEM transport is being used.

#### Sample Statistics

The number of samples received, sent, and lost per experiment run are also logged.

#### Assumptions / Known Limits

* Communication frameworks like DDS have a huge amount of settings. These are at the moment
hardcoded, aside from the common QOS settings which can be set on the command line.
* Only one publisher per topic is allowed, because the data verification logic does not support
matching data to the different publishers.
* Some communication plugins can get stuck in their internal loops if too much data is received.
But figuring out ways around such issues is one of the goals of this tool.
* ROS 2 msg files are not automatically converted to IDL files used by
the tool. But as ROS 2 will support IDL files in the near future, this issue
will be resolved.
* In Inter process composition the CPU and Resident Memory measurements are logged separately for the publisher and subscriber processes.
* FastRTPS waitset does not support a timeout which can lead to the receiving not aborting. In that case the performance test must be manually killed.
* Using Connext Micro INTRA transport with `reliable` QoS and history kind set to `keep_all` [is not supported with Connext Micro](https://community.rti.com/static/documentation/connext-micro/3.0.2/doc/html/usersmanual/transports/INTRA.html#reliability-and-durability). Set `keep_last` as QoS history kind always when using `reliable`, e.g:

```
./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t PointCloud1m --max_runtime 30 --reliable --keep_last
```

#### Architecture

The tool consists of the following modules:

##### Experiment configuration

This modules is responsible for reading the experiment configuration like
what mean of communication to use, what QOS to use and how fast data should
be transferred.

##### Experiment execution

Runs the experiment on the highest level. Responsible for setting up the experiment,
collecting experiment results regularly and outputting it to the user in form of command line
output and file output.

##### Data running

Responsible for starting the threads effectively running the experiment and synchronizing data between
the experiment threads and the experiment execution.

##### Communication Abstractions

Plugins for the various means of communications which do the actual publishing and subscribing of data.
These are used by the data running module then.

##### Interface specification files and their generation

The ROS 2 MSG and IDL files and the machinery to generate sources from
them are located there.

##### Utilities

General helper files for real time and statistics are located there.

#### Error Detection and Handling

All errors are translated to exceptions. But exception are not handled and will
propagate up causing the application to terminate.

#### Security Considerations
<!-- Required -->
The security of this tool depends mostly on the tested means of communications and their
implementation in the plugins. Therefore this tool is not secure.

If you run the tool with the `helper_scripts/run_experiment.py` script is ran as root to
gain privileges to set real time priorities. If you are not in a secure network, you should not run this too,
or at least not as root.

#### Future Extensions / Unimplemented Parts

Possible additional communication which could be implemented are:

* Raw UDP communication
