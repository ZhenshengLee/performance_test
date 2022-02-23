# Copyright 2022 Apex.AI, Inc.
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

import os
import yaml

from .logs import getExperiments
from .utils import PerfArgParser


def generate_commands(files: "list[str]", perf_test_exe_cmd, output_dir):
    commands = []
    commands.append(f'mkdir -p {output_dir}')
    for run_file in files:
        with open(run_file, "r") as f:
            run_cfg = yaml.load(f, Loader=yaml.FullLoader)

        run_configs = getExperiments(run_cfg["experiments"])

        for run_config in run_configs:
            commands += run_config.cli_commands(perf_test_exe_cmd, output_dir)
    return commands


def main():
    parser = PerfArgParser()
    parser.init_args()
    args = parser.parse_args()

    log_dir = getattr(args, "log_dir")
    test_name = getattr(args, "test_name")
    run_files = getattr(args, "configs")
    perf_test_exe_cmd = getattr(args, "perf_test_exe")

    log_dir = os.path.join(log_dir, test_name)
    commands = generate_commands(run_files, perf_test_exe_cmd, log_dir)
    print('\n'.join(commands))


# if this file is called directly
if __name__ == "__main__":
    main()
