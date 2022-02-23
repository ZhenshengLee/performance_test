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
from enum import Enum


class DURABILITY(str, Enum):
    VOLATILE = "VOLATILE"
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"

    def __str__(self):
        if self == DURABILITY.VOLATILE:
            return "V"
        if self == DURABILITY.TRANSIENT_LOCAL:
            return "T"


class HISTORY(str, Enum):
    KEEP_ALL = "KEEP_ALL"
    KEEP_LAST = "KEEP_LAST"

    def __str__(self):
        if self == HISTORY.KEEP_ALL:
            return "A"
        if self == HISTORY.KEEP_LAST:
            return "L"


class RELIABILITY(str, Enum):
    RELIABLE = "RELIABLE"
    BEST_EFFORT = "BEST_EFFORT"

    def __str__(self):
        if self == RELIABILITY.RELIABLE:
            return "R"
        if self == RELIABILITY.BEST_EFFORT:
            return "B"
