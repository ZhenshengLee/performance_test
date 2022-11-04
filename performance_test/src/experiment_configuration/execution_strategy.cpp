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

#include "execution_strategy.hpp"

#include <string>

namespace performance_test
{

std::string to_string(const ExecutionStrategy x)
{
  if (x == ExecutionStrategy::INTER_THREAD) {
    return "INTER_THREAD";
  }
  if (x == ExecutionStrategy::INTRA_THREAD) {
    return "INTRA_THREAD";
  }
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  if (x == ExecutionStrategy::APEX_SINGLE_EXECUTOR) {
    return "APEX_SINGLE_EXECUTOR";
  }
  if (x == ExecutionStrategy::APEX_EXECUTOR_PER_COMMUNICATOR) {
    return "APEX_EXECUTOR_PER_COMMUNICATOR";
  }
  if (x == ExecutionStrategy::APEX_CHAIN) {
    return "APEX_CHAIN";
  }
#endif
  throw std::invalid_argument("Enum value not supported!");
}

ExecutionStrategy execution_strategy_from_string(const std::string & s)
{
  if (s == "INTER_THREAD") {
    return ExecutionStrategy::INTER_THREAD;
  }
  if (s == "INTRA_THREAD") {
    return ExecutionStrategy::INTRA_THREAD;
  }
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  if (s == "APEX_SINGLE_EXECUTOR") {
    return ExecutionStrategy::APEX_SINGLE_EXECUTOR;
  }
  if (s == "APEX_EXECUTOR_PER_COMMUNICATOR") {
    return ExecutionStrategy::APEX_EXECUTOR_PER_COMMUNICATOR;
  }
  if (s == "APEX_CHAIN") {
    return ExecutionStrategy::APEX_CHAIN;
  }
#endif
  throw std::invalid_argument("Invalid execution strategy string!");
}

}  // namespace performance_test
