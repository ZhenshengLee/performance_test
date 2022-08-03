// Copyright 2017-2022 Apex.AI, Inc.
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

#ifndef EXPERIMENT_METRICS__PUBLISHER_STATS_HPP_
#define EXPERIMENT_METRICS__PUBLISHER_STATS_HPP_

#include <chrono>
#include <memory>

#include "analysis_result.hpp"
#include "../utilities/spin_lock.hpp"

namespace performance_test
{
struct PublisherStats
{
  std::uint64_t next_sample_id()
  {
    // Pre-increment, so the first sample ID is 1.
    // If a sample ID is ever 0, then the sample has not been initialized.
    return ++m_prev_sample_id;
  }

  void update_publisher_stats()
  {
    lock();
    increment_sent();
    unlock();
  }

  void update_stats(std::chrono::duration<double> iteration_duration)
  {
    lock();
    m_sent_samples_per_iteration =
      static_cast<decltype(m_sent_samples_per_iteration)>(
      static_cast<double>(m_sent_sample_counter) /
      iteration_duration.count());
    m_sent_sample_counter = 0;
    unlock();
  }

  void populate_stats(std::shared_ptr<AnalysisResult> & results)
  {
    lock();
    results->m_num_samples_sent += m_sent_samples_per_iteration;
    unlock();
  }

private:
  void lock()
  {
    m_lock.lock();
  }

  void unlock()
  {
    m_lock.unlock();
  }

  void increment_sent()
  {
    m_sent_sample_counter++;
  }

  std::uint64_t m_sent_sample_counter{};
  std::uint64_t m_sent_samples_per_iteration{};
  std::uint64_t m_prev_sample_id{};
  SpinLock m_lock;
};
}  // namespace performance_test

#endif  // EXPERIMENT_METRICS__PUBLISHER_STATS_HPP_
