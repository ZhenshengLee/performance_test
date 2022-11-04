// Copyright 2022 Apex.AI, Inc.
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

#ifndef EXPERIMENT_CONFIGURATION__EXECUTION_STRATEGY_HPP_
#define EXPERIMENT_CONFIGURATION__EXECUTION_STRATEGY_HPP_

#include <string>
#include <ostream>

namespace performance_test
{
enum class ExecutionStrategy
{
  INTER_THREAD,
  INTRA_THREAD,
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  APEX_SINGLE_EXECUTOR,
  APEX_EXECUTOR_PER_COMMUNICATOR,
  APEX_CHAIN,
#endif
  INVALID
};

std::string to_string(const ExecutionStrategy x);

inline std::ostream & operator<<(std::ostream & stream, const ExecutionStrategy x)
{
  return stream << to_string(x);
}

ExecutionStrategy execution_strategy_from_string(const std::string & s);

}  // namespace performance_test

#endif  // EXPERIMENT_CONFIGURATION__EXECUTION_STRATEGY_HPP_
