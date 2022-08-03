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

#ifndef UTILITIES__MEMORY_CHECKER_HPP_
#define UTILITIES__MEMORY_CHECKER_HPP_

#include <string>

#include "../experiment_configuration/experiment_configuration.hpp"

namespace performance_test
{
class MemoryChecker
{
public:
  MemoryChecker(const MemoryChecker &) = delete;
  MemoryChecker & operator=(const MemoryChecker &) = delete;
  explicit MemoryChecker(const ExperimentConfiguration & ec);
  virtual ~MemoryChecker() = default;

  void enable_memory_tools_checker();

private:
  bool m_memory_tools_on;

  void malloc_test_function(const std::string & str);

#ifdef PERFORMANCE_TEST_MEMORYTOOLS_ENABLED
  void assert_memory_tools_is_working();
#endif
};

}  // namespace performance_test
#endif  // UTILITIES__MEMORY_CHECKER_HPP_
