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

#ifndef DATA_RUNNING__DATA_ENTITY_HPP_
#define DATA_RUNNING__DATA_ENTITY_HPP_

#include <string>

#ifdef PERFORMANCE_TEST_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>
#endif

#include "../communication_abstractions/communicator.hpp"
#include "../experiment_configuration/experiment_configuration.hpp"
#include "../utilities/perf_clock.hpp"
#include "../utilities/statistics_tracker.hpp"

namespace performance_test
{

/// Interface for generic object which effectively executes the experiment and collects the
/// experiment results.
class DataEntity
{
public:
  DataEntity(const DataEntity &) = delete;
  DataEntity & operator=(const DataEntity &) = delete;

  DataEntity()
  : m_ec(ExperimentConfiguration::get())
  {
    if (m_ec.check_memory()) {
#ifdef PERFORMANCE_TEST_MEMORYTOOLS_ENABLED
      osrf_testing_tools_cpp::memory_tools::initialize();
      osrf_testing_tools_cpp::memory_tools::enable_monitoring_in_all_threads();
      assert_memory_tools_is_working();
      const auto on_unexpected_memory =
        [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService & service) {
          // this will cause a backtrace to be printed for each unexpected memory operations
          service.print_backtrace();
        };
      osrf_testing_tools_cpp::memory_tools::on_unexpected_calloc(on_unexpected_memory);
      osrf_testing_tools_cpp::memory_tools::on_unexpected_free(on_unexpected_memory);
      osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(on_unexpected_memory);
      osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(on_unexpected_memory);

#else
      throw std::runtime_error(
              "OSRF memory tools is not installed. Memory check must be disabled.");
#endif
    }
  }
  virtual ~DataEntity() = default;

  virtual void run() = 0;

protected:
  /// A reference to the experiment configuration.
  const ExperimentConfiguration & m_ec;

  bool m_memory_tools_on = false;

  void enable_memory_tools_checker()
  {
#ifdef PERFORMANCE_TEST_MEMORYTOOLS_ENABLED
    // Do not turn the memory tools on several times.
    if (m_memory_tools_on) {
      return;
    }

    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_free_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_begin();

    m_memory_tools_on = true;
#endif
  }

  void malloc_test_function(const std::string & str)
  {
    void * some_memory = std::malloc(1024);
    // We need to do something with the malloc'ed memory to make sure this
    // function doesn't get optimized away.  memset isn't enough, so we do a
    // memcpy from a passed in string, which is enough to keep the optimizer away.
    // see: https://github.com/osrf/osrf_testing_tools_cpp/pull/8
    memcpy(some_memory, str.c_str(), str.length());
    std::free(some_memory);
  }

#ifdef PERFORMANCE_TEST_MEMORYTOOLS_ENABLED
  void assert_memory_tools_is_working()
  {
    bool saw_malloc = false;
    auto on_malloc_cb = [&saw_malloc]() {
        saw_malloc = true;
      };
    osrf_testing_tools_cpp::memory_tools::on_malloc(on_malloc_cb);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(osrf_testing_tools_cpp::memory_tools::on_malloc(nullptr));
    {
      const std::string test_str = "test message";
      malloc_test_function(test_str);
    }
    if (!saw_malloc) {
      throw std::runtime_error(
              "Memory checking does not work properly. Please consult the documentation on how to "
              "properly set it up.");
    }
  }
#endif
};

}  // namespace performance_test
#endif  // DATA_RUNNING__DATA_ENTITY_HPP_
