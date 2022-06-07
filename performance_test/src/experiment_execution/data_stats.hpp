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

#ifndef EXPERIMENT_EXECUTION__DATA_STATS_HPP_
#define EXPERIMENT_EXECUTION__DATA_STATS_HPP_

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
#include "run_type.hpp"

namespace performance_test
{

struct DataStats
{
  DataStats()
  {
#if defined(QNX)
    m_cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
#endif
  }

  void reset()
  {
    m_pub_loop_time_reserves.clear();
    m_latencies.clear();
  }

  /**
   * \brief Increment the number of received samples.
   * \param increment Optional different increment step.
   */
  void increment_received(const std::uint64_t & increment = 1)
  {
    m_received_sample_counter += increment;
  }

  /**
   * \brief Increment the number of sent samples.
   * \param increment Optional different increment step.
   */
  void increment_sent(const std::uint64_t & increment = 1)
  {
    m_sent_sample_counter += increment;
  }

  std::uint64_t num_sent_samples() const {return m_sent_sample_counter;}

  /**
   * \brief Given a sample id this function check if and how many samples were
   * lost and updates counters accordingly. \param sample_id The sample id to
   * check.
   */
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

  void update_data_received(const std::size_t data_type_size)
  {
    m_data_received_bytes = m_received_sample_counter * data_type_size;
  }

  /// Get the the id for the next sample to publish.
  std::uint64_t next_sample_id()
  {
    /* We never send a sample with id 0 to make sure we not just dealing with
       default constructed samples.
     */
    m_prev_sample_id = m_prev_sample_id + 1;
    return m_prev_sample_id;
  }

  void update_subscriber_stats(
    const std::int64_t time_ns_since_epoch,
    const std::uint64_t sample_id,
    const std::size_t data_type_size)
  {
    update_lost_samples_counter(sample_id);
    add_latency_to_statistics(time_ns_since_epoch);
    increment_received();
    update_data_received(data_type_size);
  }

  void update_publisher_stats(
    const std::chrono::duration<double> & remaining_time_to_publish)
  {
    increment_sent();
    m_pub_loop_time_reserves.push_back(remaining_time_to_publish.count());
  }

  void lock() {m_lock.lock();}

  void unlock() {m_lock.unlock();}

  void update_stats(
    RunType run_type,
    std::chrono::duration<double> iteration_duration)
  {
    m_lock.lock();
    switch (run_type) {
      case RunType::PUBLISHER:
        m_sent_samples_per_iteration =
          static_cast<decltype(m_sent_samples_per_iteration)>(
          static_cast<double>(m_sent_sample_counter) /
          iteration_duration.count());
        m_sent_sample_counter = 0;
        break;
      case RunType::SUBSCRIBER:
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
        break;
    }
    m_lock.unlock();
  }

  void populate_stats(
    RunType run_type,
    std::shared_ptr<AnalysisResult> & results)
  {
    m_lock.lock();
    switch (run_type) {
      case RunType::PUBLISHER:
        results->m_num_samples_sent += m_sent_samples_per_iteration;
        for (auto reserve : m_pub_loop_time_reserves) {
          results->m_pub_loop_time_reserve.add_sample(reserve);
        }
        break;
      case RunType::SUBSCRIBER:
        results->m_num_samples_received += m_received_samples_per_iteration;
        results->m_num_samples_lost += m_lost_samples_per_iteration;
        results->m_total_data_received += m_received_data_per_iteration;
        for (auto latency : m_latencies) {
          results->m_latency.add_sample(latency);
        }
        break;
    }

    m_lock.unlock();
  }

  /**
   * \brief Adds a sample timestamp to the latency statistics.
   * \param sample_timestamp The timestamp the sample was sent.
   */
  void add_latency_to_statistics(const std::int64_t sample_timestamp)
  {
#if defined(QNX)
    // Since clock resolution on QNX is 1ms or above, it is better to use
    // clock cycles to calculate latencies instead of CLOCK_REALTIME or
    // CLOCK_MONOTONIC.
    std::uint64_t rclk_cyc = ClockCycles();
    std::uint64_t sclk_cyc = static_cast<std::uint64_t>(sample_timestamp);
    std::uint64_t ncycles = (rclk_cyc - sclk_cyc);
    // std::uint64_t m_cpms = (m_cps/1000);
    // std::uint64_t m_cpus = (m_cpms/1000);
    const double sec_diff =
      static_cast<double>(ncycles) / static_cast<double>(m_cps);
#else
    std::chrono::nanoseconds st(sample_timestamp);
    const auto diff = std::chrono::steady_clock::now().time_since_epoch() - st;

    const auto sec_diff =
      std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
#endif
    // Converting to double for easier calculations. Because the two
    // timestamps are very close double precision is enough.
    m_latencies.push_back(sec_diff);
  }

  std::vector<double> m_pub_loop_time_reserves;

private:
  std::vector<double> m_latencies;
  std::uint64_t m_sent_sample_counter{};
  std::uint64_t m_received_sample_counter{};
  std::uint64_t m_num_lost_samples{};
  std::size_t m_received_data_per_iteration{};
  std::uint64_t m_sent_samples_per_iteration{};
  std::uint64_t m_received_samples_per_iteration{};
  std::uint64_t m_lost_samples_per_iteration{};
  std::uint64_t m_prev_sample_id{};
  std::size_t m_data_received_bytes{};
  SpinLock m_lock;

#if defined(QNX)
  std::uint64_t m_cps;
#endif
};
}  // namespace performance_test

#endif  // EXPERIMENT_EXECUTION__DATA_STATS_HPP_