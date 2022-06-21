# Copyright 2017-2022 Apex.AI, Inc., Arm Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Script to run a batch of performance experiments."""
import argparse
from dataclasses import dataclass
from enum import Enum
import itertools
import os
import signal
import subprocess
import sys


class LiteralEnum(Enum):
    """Enum represented by the literal name."""

    def __repr__(self):
        return self._name_

    def __str__(self):
        return self.__repr__()


class ValueEnum(Enum):
    """Enum represented by the value."""

    def __repr__(self):
        return self.value

    def __str__(self):
        return self.__repr__()


class InstanceType(ValueEnum):
    """Define the enumeration for the experiment types."""

    PUBLISHER = 'pub'
    SUBSCRIBER = 'sub'
    PUBSUB = 'pubsub'

    def to_cmd_args(self, test_args):
        args = []

        if self == InstanceType.SUBSCRIBER:
            args.append('-p 0')
        else:
            args.append('-p 1')

        if self != InstanceType.PUBLISHER:
            args.append('-s {}'.format(test_args.subscribers))

        return args


class OutputType(ValueEnum):
    """Define the enumeration for the output types."""

    CSV = "csv"
    JSON = "json"

    def to_cmd_args(self, test_args):
        filepath = test_args.directory + test_args.filename
        return '-l {}.{}'.format(filepath, self)


def get_flag_from_holder(self, flag_holder, flag):
    args = []
    if self == flag_holder:
        args.append('--{}'.format(flag))
    return args


class Reliability(LiteralEnum):
    """Define the enumeration for the reliability types."""

    BEST_EFFORT = 0
    RELIABLE = 1

    def to_cmd_args(self):
        return get_flag_from_holder(self, Reliability.RELIABLE, 'reliable')


class Durability(LiteralEnum):
    """Define the enumeration for the durability types."""

    VOLATILE = 0
    TRANSIENT_LOCAL = 1

    def to_cmd_args(self):
        return get_flag_from_holder(
            self,
            Durability.TRANSIENT_LOCAL,
            'transient')


class MessageType(ValueEnum):
    """Define the enumeration for the message types."""

    ARRAY_1K = 'Array1k'
    ARRAY_4K = 'Array4k'
    ARRAY_16K = 'Array16k'
    ARRAY_32K = 'Array32k'
    ARRAY_60K = 'Array60k'
    ARRAY_64K = 'Array64k'
    ARRAY_256K = 'Array256k'
    ARRAY_1M = 'Array1m'
    ARRAY_2M = 'Array2m'
    STRUCT_16 = 'Struct16'
    STRUCT_256 = 'Struct256'
    STRUCT_4K = 'Struct4k'
    STRUCT_32K = 'Struct32k'
    POINT_CLOUD_512K = 'PointCloud512k'
    POINT_CLOUD_1M = 'PointCloud1m'
    POINT_CLOUD_2M = 'PointCloud2m'
    POINT_CLOUD_4M = 'PointCloud4m'
    RANGE = 'Range'
    NAV_SAT_FIX = 'NavSatFix'
    RADAR_DETECTION = 'RadarDetection'
    RADAR_TRACK = 'RadarTrack'

    def to_cmd_args(self):
        return ['--msg {}'.format(self.value)]


@dataclass
class TestArguments:
    """Define the struct for the test arguments."""

    output_type: OutputType
    rate: int
    subscribers: int
    middleware: str
    message: MessageType
    reliability: Reliability
    durability: Durability
    duration: int
    test_type: InstanceType = InstanceType.PUBSUB

    @property
    def directory(self):
        return './rate_{rate}/subs_{subs}/'.format(
            rate=self.rate,
            subs=self.subscribers,
        )

    @property
    def filename(self):
        return '{msg}_{mw}_{type}_{rel}_{dur}'.format(
            msg=self.message,
            mw=self.middleware,
            type=self.test_type,
            rel=self.reliability,
            dur=self.durability,
        )

    @classmethod
    def unpack(cls, args):
        return cls(*args)

    def to_cmd_args(self):
        args = []

        args.append('--rate {}'.format(self.rate))
        args.append('-c {}'.format(self.middleware))
        args.append('--max-runtime {}'.format(self.duration))
        args.extend(self.output_type.to_cmd_args())
        args.extend(self.message.to_cmd_args())
        args.extend(self.reliability.to_cmd_args())
        args.extend(self.durability.to_cmd_args())
        args.extend(self.test_type.to_cmd_args(self))

        return args


class Instance:
    """perf_test process encapsulation."""

    def __init__(self, operation_type, tests_args, args, pout):
        """
        Construct the object.

        :param operation_type: Type of the operation
        :param tests_args: Tests arguments
        :param args: User arguments object
        :param pout: process stdout
        """
        self.type = operation_type
        self.tests_args = tests_args
        self.args = args
        self.pout = pout

        self.process = None

    def run(self, index):
        """
        Run the embedded perf_test process.

        :param index: The test configuration to run.
        """
        if self.args.v:
            print('*******************')
            print(self.cmd(index))
            print('*******************')
        else:
            sys.stdout.flush()

        env_vars = os.environ.copy()
        if self.args.rclcpp_middleware:
            env_vars['RMW_IMPLEMENTATION'] = self.args.rclcpp_middleware

        self.process = subprocess.Popen(
            self.cmd(index), shell=True, env=env_vars,
            stdout=pout,
            stderr=pout,
            preexec_fn=os.setsid)

        # Comment out the following lines to run the experiments
        # with soft realtime priority.
        # We sleeping here to make sure the process is started before
        # changing its priority.
        # time.sleep(2)
        # Enabling (pseudo-)realtime
        # subprocess.Popen(
        #     'chrt -p 99 $(ps -o pid -C 'perf_test' --no-headers)', shell=True
        # )

    def cmd(self, index):
        """
        Return the command line necessary to execute the performance test.

        :param index: The test configuration the returned command line should
            contain.
        :return: The command line argument to execute the performance test.
        """
        command = 'ros2 run performance_test perf_test'

        test_args = self.tests_args[index]
        test_args.test_type = self.type

        if not os.path.exists(test_args.directory):
            os.makedirs(test_args.directory)

        return ' '.join([command] + test_args.to_cmd_args())

    def kill(self):
        """Kill the associated performance test process."""
        if self.process is not None:
            ret_code = self.process.poll()
            if ret_code is None:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait()
                return True
            if ret_code == 0:
                return True
        return False

    def num_runs(self):
        """Return the number of experiments runs this instance can execute."""
        return len(self.tests_args)

    def __del__(self):
        """Kill the associated performance test process."""
        self.kill()


"""Script arguments defaults"""
defaults = {
    'TEST_DURATION': 120,
    'OUTPUT_TYPE': [OutputType.CSV],
    'MESSAGES': list(MessageType),
    'RELIABILITY': [Reliability.BEST_EFFORT],
    'DURABILITY': [Durability.VOLATILE],
    'RATES': [20, 50, 1000],
    'SUBSCRIBERS': [1, 3, 10],
}

"""Script arguments"""
parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '-c',
    '--communication',
    nargs='+',
    required=True,
    help='perfomance_test communication middlewares to use it')
parser.add_argument(
    '--rclcpp-middleware',
    help='ROS2 middleware, to set when the ROS2 rclcpp API is ' +
         'used as middleware')
parser.add_argument(
    '-d',
    '--duration',
    default=defaults['TEST_DURATION'],
    type=int,
    help='runtime of each test in seconds')
parser.add_argument(
    '-o',
    '--output',
    nargs='*',
    type=OutputType,
    choices=OutputType,
    help='log output type')
parser.add_argument(
    '-m',
    '--msg',
    nargs='*',
    type=MessageType,
    default=defaults['MESSAGES'],
    choices=MessageType,
    help='list of messages to test')
parser.add_argument(
    '-s',
    '--subscribers',
    nargs='*',
    default=defaults['SUBSCRIBERS'],
    type=int,
    choices=defaults['SUBSCRIBERS'],
    help='number of subscribers used to run the tests')
parser.add_argument(
    '-r',
    '--rates',
    nargs='*',
    default=defaults['RATES'],
    type=int,
    choices=defaults['RATES'],
    help='number of rates used to run the tests')
parser.add_argument(
    '--reliability',
    nargs='*',
    type=Reliability.__getitem__,
    default=defaults['RELIABILITY'],
    choices=Reliability,
    help='list of reliability options to test')
parser.add_argument(
    '--durability',
    nargs='*',
    type=Durability.__getitem__,
    default=defaults['DURABILITY'],
    choices=Durability,
    help='list of durability options to test')
parser.add_argument(
    '--pub-procs',
    type=int,
    default=0,
    help='publisher-only processes to run')
parser.add_argument(
    '--sub-procs',
    type=int,
    default=0,
    help='subscriber-only processes to run')
parser.add_argument(
    '--pubsub-procs',
    type=int,
    default=1,
    help='publisher and subscriber processes to run')
parser.add_argument(
    '-v',
    action='store_const',
    const=True,
    help='verbose mode')

args = parser.parse_args()

"""Product of all the test arguments"""
tests_args = list(map(TestArguments.unpack, itertools.product(
    [args.output],
    args.rates,
    args.subscribers,
    args.communication,
    args.msg,
    args.reliability,
    args.durability,
    [args.duration],
)))

"""Process stdout"""
pout = subprocess.DEVNULL
if args.v:
    pout = None

current_index = 0

num_pub_processes = args.pub_procs
num_sub_processes = args.sub_procs
num_both = args.pubsub_procs

pub_list = [Instance(InstanceType.PUBLISHER, tests_args, args, pout)
            for _ in range(0, num_pub_processes)]
sub_list = [Instance(InstanceType.SUBSCRIBER, tests_args, args, pout)
            for _ in range(0, num_sub_processes)]
both_list = [Instance(InstanceType.PUBSUB, tests_args, args, pout)
             for _ in range(0, num_both)]
full_list = pub_list + sub_list + both_list


def signal_handler(sig, frame):
    """Signal handler to handle Ctrl-C."""
    print()
    print('You pressed Ctrl+C! Terminating experiment')
    subprocess.Popen('killall perf_test', shell=True, stdout=pout, stderr=pout)
    sys.exit(0)


def timer_handler(sig=None, frame=None):
    """Signal handler to handle the timer."""
    global current_index
    global full_list

    failed = False in [e.kill() for e in full_list]

    if not args.v and current_index > 0:
        if failed:
            print('fail')
        else:
            print('ok')

    if current_index >= full_list[0].num_runs():
        if args.v:
            print('Done with experiments.')
        exit(0)
    else:
        test_args = tests_args[current_index]
        msg = '[{}/{}] {}, rate {}, subs {}'.format(
            current_index + 1,
            len(tests_args),
            test_args.message.value,
            test_args.rate,
            test_args.subscribers)
        if test_args.reliability == Reliability.RELIABLE:
            msg += ', reliable'
        if test_args.durability == Durability.TRANSIENT_LOCAL:
            msg += ', transient'
        print(msg, end='...')
        if args.v:
            print()

        [e.run(current_index) for e in full_list]
    current_index = current_index + 1


signal.signal(signal.SIGALRM, timer_handler)
signal.signal(signal.SIGINT, signal_handler)
signal.setitimer(signal.ITIMER_REAL, args.duration, args.duration)

estimated_runtime = len(tests_args) * len(full_list) * args.duration

print(
    'Running {} tests for an estimated runtime of {}s'.format(
        len(tests_args),
        estimated_runtime))
print('Press Ctrl+C to abort experiment')
print()

timer_handler()
while True:
    signal.pause()
    if args.v:
        print('Next experiment.')
