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

#ifndef EXPERIMENT_METRICS__SUBSCRIBER_STATS_HPP_
#define EXPERIMENT_METRICS__SUBSCRIBER_STATS_HPP_

#if defined(QNX)
#include <inttypes.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>
#endif

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "../experiment_configuration/experiment_configuration.hpp"
#include "../utilities/spin_lock.hpp"
#include "../utilities/statistics_tracker.hpp"
#include "../utilities/perf_clock.hpp"

namespace performance_test
{
struct ReceivedMsgStats
{
  std::int64_t time_msg_sent_ns;
  std::int64_t time_msg_received_ns;
  std::uint64_t sample_id;
  std::size_t data_type_size;

  ReceivedMsgStats(
    std::int64_t time_msg_sent_ns,
    std::int64_t time_msg_received_ns,
    std::uint64_t sample_id,
    std::size_t data_type_size
  )
  : time_msg_sent_ns(time_msg_sent_ns),
    time_msg_received_ns(time_msg_received_ns),
    sample_id(sample_id),
    data_type_size(data_type_size)
  {}
};

struct SubscriberStats
{
  SubscriberStats()
  {
#if defined(QNX)
    m_cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
#endif
  }

  void reset()
  {
    m_latencies.clear();
  }

  void update_subscriber_stats(const ReceivedMsgStats & stats)
  {
    update_subscriber_stats(
      stats.time_msg_sent_ns,
      stats.time_msg_received_ns,
      stats.sample_id,
      stats.data_type_size
    );
  }

  void update_subscriber_stats(
    const std::int64_t time_msg_sent_ns,
    const std::int64_t time_msg_received_ns,
    const std::uint64_t sample_id,
    const std::size_t data_type_size)
  {
    lock();
    update_lost_samples_counter(sample_id);
    add_latency_to_statistics(time_msg_sent_ns, time_msg_received_ns);
    increment_received();
    update_data_received(data_type_size);
    unlock();
  }

  void update_stats(std::chrono::duration<double> iteration_duration)
  {
    lock();
    m_received_samples_per_iteration =
      static_cast<decltype(m_received_samples_per_iteration)>(
      static_cast<double>(m_received_sample_counter) /
      iteration_duration.count());
    m_received_data_per_iteration =
      static_cast<decltype(m_received_data_per_iteration)>(
      static_cast<double>(m_data_received_bytes) /
      iteration_duration.count());
    m_lost_samples_per_iteration =
      static_cast<decltype(m_lost_samples_per_iteration)>(
      static_cast<double>(m_num_lost_samples) /
      iteration_duration.count());
    m_received_sample_counter = 0;
    m_num_lost_samples = 0;
    unlock();
  }

  void populate_stats(std::shared_ptr<AnalysisResult> & results)
  {
    lock();
    results->m_num_samples_received += m_received_samples_per_iteration;
    results->m_num_samples_lost += m_lost_samples_per_iteration;
    results->m_total_data_received += m_received_data_per_iteration;
    for (auto latency : m_latencies) {
      results->m_latency.add_sample(latency);
    }
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

  void verify_sample_chronological_order(std::int64_t time_ns_since_epoch)
  {
    if (m_prev_timestamp_ns_since_epoch >= time_ns_since_epoch) {
      throw std::runtime_error(
              "Data not consistent: received sample with not strictly older "
              "timestamp. Time diff: " +
              std::to_string(time_ns_since_epoch - m_prev_timestamp_ns_since_epoch) +
              " Data Time: " + std::to_string(time_ns_since_epoch));
    }
    m_prev_timestamp_ns_since_epoch = time_ns_since_epoch;
  }

  void update_lost_samples_counter(const std::uint64_t sample_id)
  {
    // We can lose samples, but samples always arrive in the right order and
    // no duplicates exist.
    if (sample_id <= m_prev_sample_id) {
      throw std::runtime_error(
              "Data not consistent: received sample with not strictly greater id."
              " Received sample id: " +
              std::to_string(sample_id) +
              " Previous sample id: " + std::to_string(m_prev_sample_id));
    }
    m_num_lost_samples += sample_id - m_prev_sample_id - 1;
    m_prev_sample_id = sample_id;
  }

  void add_latency_to_statistics(
    const std::int64_t time_msg_sent_ns,
    const std::int64_t time_msg_received_ns)
  {
#if defined(QNX)
    // Since clock resolution on QNX is 1ms or above, it is better to use
    // clock cycles to calculate latencies instead of CLOCK_REALTIME or
    // CLOCK_MONOTONIC.
    const double ncycles =
      static_cast<double>(time_msg_received_ns) - static_cast<double>(time_msg_sent_ns);
    const double latency_s =
      static_cast<double>(ncycles) / static_cast<double>(m_cps);
#else
    std::chrono::nanoseconds time_msg_sent(time_msg_sent_ns);
    std::chrono::nanoseconds time_msg_received(time_msg_received_ns);
    const auto latency = time_msg_received - time_msg_sent;
    const auto latency_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(latency).count();
#endif
    // Converting to double for easier calculations. Because the two
    // timestamps are very close double precision is enough.
    m_latencies.push_back(latency_s);
  }

  void increment_received()
  {
    m_received_sample_counter++;
  }

  void update_data_received(const std::size_t data_type_size)
  {
    m_data_received_bytes = m_received_sample_counter * data_type_size;
  }

  std::vector<double> m_latencies;
  std::uint64_t m_received_sample_counter{};
  std::uint64_t m_num_lost_samples{};
  std::size_t m_received_data_per_iteration{};
  std::uint64_t m_received_samples_per_iteration{};
  std::uint64_t m_lost_samples_per_iteration{};
  std::uint64_t m_prev_sample_id{};
  std::size_t m_data_received_bytes{};
  SpinLock m_lock;
  std::int64_t m_prev_timestamp_ns_since_epoch{};

#if defined(QNX)
  std::uint64_t m_cps;
#endif
};
}  // namespace performance_test

#endif  // EXPERIMENT_METRICS__SUBSCRIBER_STATS_HPP_
