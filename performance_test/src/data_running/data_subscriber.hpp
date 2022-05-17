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

#ifndef DATA_RUNNING__DATA_SUBSCRIBER_HPP_
#define DATA_RUNNING__DATA_SUBSCRIBER_HPP_

#include "data_entity.hpp"
#ifdef PERFORMANCE_TEST_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#endif
#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#if defined(QNX)
#include <inttypes.h>
#include <sys/neutrino.h>
#endif

#include "../utilities/spin_lock.hpp"

namespace performance_test
{

/// Generic class which effectively executes the experiment and collects the
/// experiment results.
template<class TCommunicator>
class DataSubscriber : public DataEntity
{
public:
  /**
   * \brief Constructs an object and starts the internal worker thread.
   */
  explicit DataSubscriber(DataStats & stats)
  : m_com(stats) {}

  DataSubscriber & operator=(const DataSubscriber &) = delete;
  DataSubscriber(const DataSubscriber &) = delete;

  void run() override
  {
    m_com.update_subscription();
    enable_memory_tools_checker();
  }

  TCommunicator m_com;
};

}  // namespace performance_test

#endif  // DATA_RUNNING__DATA_SUBSCRIBER_HPP_
