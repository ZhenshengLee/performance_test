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
from performance_report.utils import generate_shmem_file_yml


def test_generate_shmem_file(tmp_path):
    generate_shmem_file_yml(tmp_path)
    shmem_config_file = os.path.join(tmp_path, "shmem.yml")
    with open(shmem_config_file, "r") as f:
        yaml_contents = yaml.load(f, Loader=yaml.FullLoader)
        assert yaml_contents["domain"]["shared_memory"]["enable"] is True
