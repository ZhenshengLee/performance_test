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

#ifndef DATA_RUNNING__DATA_PUBLISHER_HPP_
#define DATA_RUNNING__DATA_PUBLISHER_HPP_

#include "data_entity.hpp"
#ifdef PERFORMANCE_TEST_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#endif
#include <atomic>
#include <memory>
#include <thread>
#include <functional>

#if defined(QNX)
#include <sys/neutrino.h>
#include <inttypes.h>
#endif

#include "../utilities/perf_clock.hpp"
#include "../utilities/spin_lock.hpp"

namespace performance_test
{

namespace chrono = std::chrono;

/// Generic class which effectively executes the experiment and collects the experiment results.
template<class TCommunicator>
class DataPublisher : public DataEntity
{
public:
  /**
   * \brief Constructs an object and starts the internal worker thread.
   */
  explicit DataPublisher(DataStats & stats)
  : m_com(stats),
    m_time_between_publish(1.0 / static_cast<double>(m_ec.rate())),
    m_first_run(perf_clock::now()),
    m_next_run(perf_clock::now() +
      chrono::duration_cast<chrono::nanoseconds>(
        m_time_between_publish)) {}

  DataPublisher & operator=(const DataPublisher &) = delete;
  DataPublisher(const DataPublisher &) = delete;

  void run() override
  {
    // We track here how much time (can also be negative) was left for the
    // loop iteration given the desired loop rate.
    const std::chrono::nanoseconds reserve = m_next_run - perf_clock::now();

    if (reserve.count() > 0 &&
      m_ec.roundtrip_mode() !=
      ExperimentConfiguration::RoundTripMode::RELAY)
    {
      std::this_thread::sleep_until(m_next_run);
    }

#if defined(QNX)
    std::int64_t epoc_time = static_cast<std::int64_t>(ClockCycles());
#else
    std::int64_t epoc_time = perf_clock::now().time_since_epoch().count();
#endif
    m_com.publish(epoc_time);

    m_next_run =
      m_first_run +
      m_loop_counter * std::chrono::duration_cast<std::chrono::nanoseconds>(
      m_time_between_publish);
    ++m_loop_counter;
    enable_memory_tools_checker();
  }

  TCommunicator m_com;
  const chrono::duration<double> m_time_between_publish;
  const perf_clock::time_point m_first_run;
  perf_clock::time_point m_next_run;
  std::size_t m_loop_counter{1};
};

}  // namespace performance_test

#endif  // DATA_RUNNING__DATA_PUBLISHER_HPP_
