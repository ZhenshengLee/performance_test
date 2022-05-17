// Copyright 2017 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "communicator.hpp"

namespace performance_test
{

Communicator::Communicator(DataStats & stats)
: m_stats(stats), m_ec(ExperimentConfiguration::get()), m_prev_timestamp()
{
}

void Communicator::check_data_consistency(std::int64_t time_ns_since_epoch)
{
  if (m_prev_timestamp >= time_ns_since_epoch) {
    throw std::runtime_error(
            "Data not consistent: received sample with not strictly older "
            "timestamp. Time diff: " +
            std::to_string(time_ns_since_epoch - m_prev_timestamp) +
            " Data Time: " + std::to_string(time_ns_since_epoch));
  }
  m_prev_timestamp = time_ns_since_epoch;
}

}  // namespace performance_test
