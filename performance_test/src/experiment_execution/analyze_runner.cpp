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
#include <algorithm>
#include <cstdlib>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "analyze_runner.hpp"
#include "analysis_result.hpp"

#ifdef QNX710
using perf_clock = std::chrono::system_clock;
#else
using perf_clock = std::chrono::steady_clock;
#endif

namespace performance_test
{

AnalyzeRunner::AnalyzeRunner()
: m_ec(ExperimentConfiguration::get()),
  m_is_first_entry(true)
{
  for (uint32_t i = 0; i < m_ec.number_of_publishers(); ++i) {
    m_pub_runners.push_back(
      DataRunnerFactory::get(m_ec.msg_name(), m_ec.com_mean(), RunType::PUBLISHER));
  }
  for (uint32_t i = 0; i < m_ec.number_of_subscribers(); ++i) {
    m_sub_runners.push_back(
      DataRunnerFactory::get(m_ec.msg_name(), m_ec.com_mean(), RunType::SUBSCRIBER));
  }
  for (const auto & output : m_ec.configured_outputs()) {
    bind_output(output);
  }
}

void AnalyzeRunner::bind_output(std::shared_ptr<Output> output)
{
  m_outputs.push_back(output);
}

void AnalyzeRunner::run()
{
  for (const auto & output : m_outputs) {
    output->open();
  }

  const auto experiment_start = perf_clock::now();

  while (!check_exit(experiment_start)) {
    const auto loop_start = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::for_each(m_pub_runners.begin(), m_pub_runners.end(), [](auto & a) {a->sync_reset();});
    std::for_each(m_sub_runners.begin(), m_sub_runners.end(), [](auto & a) {a->sync_reset();});

#if PERFORMANCE_TEST_RT_ENABLED
    /// If there are custom RT settings and this is the first loop, set the post
    /// RT init settings
    if (m_is_first_entry && m_ec.is_rt_init_required()) {
      post_proc_rt_init();
      m_is_first_entry = false;
    }
#endif

    auto now = perf_clock::now();
    auto loop_diff_start = now - loop_start;
    auto experiment_diff_start = now - experiment_start;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(experiment_diff_start).count();
    if (seconds > m_ec.rows_to_ignore()) {
      auto result = analyze(loop_diff_start, experiment_diff_start);
      for (const auto & output : m_outputs) {
        output->update(result);
      }
    }
  }

  for (const auto & output : m_outputs) {
    output->close();
  }
}

std::shared_ptr<const AnalysisResult> AnalyzeRunner::analyze(
  const std::chrono::nanoseconds loop_diff_start,
  const std::chrono::nanoseconds experiment_diff_start)
{
  std::vector<StatisticsTracker> latency_vec(m_sub_runners.size());
  std::transform(
    m_sub_runners.begin(), m_sub_runners.end(), latency_vec.begin(),
    [](const auto & a) {return a->latency_statistics();});

  std::vector<StatisticsTracker> ltr_pub_vec(m_pub_runners.size());
  std::transform(
    m_pub_runners.begin(), m_pub_runners.end(), ltr_pub_vec.begin(),
    [](const auto & a) {return a->loop_time_reserve_statistics();});

  std::vector<StatisticsTracker> ltr_sub_vec(m_sub_runners.size());
  std::transform(
    m_sub_runners.begin(), m_sub_runners.end(), ltr_sub_vec.begin(),
    [](const auto & a) {return a->loop_time_reserve_statistics();});

  uint64_t sum_received_samples = 0;
  for (auto e : m_sub_runners) {
    sum_received_samples += e->sum_received_samples();
  }

  uint64_t sum_sent_samples = 0;
  for (auto e : m_pub_runners) {
    sum_sent_samples += e->sum_sent_samples();
  }

  uint64_t sum_lost_samples = 0;
  for (auto e : m_sub_runners) {
    sum_lost_samples += e->sum_lost_samples();
  }

  uint64_t sum_data_received = 0;
  for (auto e : m_sub_runners) {
    sum_data_received += e->sum_data_received();
  }

  auto result = std::make_shared<const AnalysisResult>(
    experiment_diff_start,
    loop_diff_start,
    sum_received_samples,
    sum_sent_samples,
    sum_lost_samples,
    sum_data_received,
    StatisticsTracker(latency_vec),
    StatisticsTracker(ltr_pub_vec),
    StatisticsTracker(ltr_sub_vec),
    cpu_usage_tracker.get_cpu_usage()
  );
  return result;
}

bool AnalyzeRunner::check_exit(std::chrono::steady_clock::time_point experiment_start) const
{
  if (m_ec.exit_requested()) {
    std::cout << "Caught signal. Exiting." << std::endl;
    return true;
  }

  if (m_ec.max_runtime() == 0) {
    // Run forever,
    return false;
  }

  const double runtime_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - experiment_start).count();

  if (runtime_sec > static_cast<double>(m_ec.max_runtime())) {
    std::cout << "Maximum runtime reached. Exiting." << std::endl;
    return true;
  } else {
    return false;
  }
}

}  // namespace performance_test
