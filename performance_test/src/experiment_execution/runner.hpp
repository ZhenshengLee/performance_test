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

#ifndef EXPERIMENT_EXECUTION__RUNNER_HPP_
#define EXPERIMENT_EXECUTION__RUNNER_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

#include "../communication_abstractions/communicator.hpp"
#include "../communication_abstractions/resource_manager.hpp"
#include "../data_running/data_entity_factory.hpp"
#include "../experiment_configuration/experiment_configuration.hpp"
#include "../outputs/output.hpp"
#include "../utilities/cpu_usage_tracker.hpp"
#include "../utilities/rt_enabler.hpp"
#include "analysis_result.hpp"

namespace performance_test
{

template<typename EntityType>
class Runner
{
public:
  explicit Runner(const ExperimentConfiguration & ec)
  : m_ec(ec), m_pub_stats(ec.number_of_publishers()),
    m_sub_stats(ec.number_of_subscribers())
  {
    for (const auto & output : ec.configured_outputs()) {
      m_outputs.push_back(output);
    }
    for (const auto & output : m_outputs) {
      output->open();
    }

    for (uint32_t i = 0; i < ec.number_of_subscribers(); ++i) {
      m_subs.push_back(
        DataEntityFactory::get(
          ec.msg_name(), ec.com_mean(),
          RunType::SUBSCRIBER,
          m_sub_stats.at(i)));
    }

    for (uint32_t i = 0; i < ec.number_of_publishers(); ++i) {
      m_pubs.push_back(
        DataEntityFactory::get(
          ec.msg_name(), ec.com_mean(), RunType::PUBLISHER, m_pub_stats.at(i)));
    }

#if PERFORMANCE_TEST_RT_ENABLED
    if (m_ec.is_rt_init_required()) {
      post_proc_rt_init();
    }
#endif  // PERFORMANCE_TEST_RT_ENABLED
  }

  ~Runner()
  {
    for (auto & thread : m_thread_pool) {
      thread.join();
    }
    for (const auto & output : m_outputs) {
      output->close();
    }

    ResourceManager::shutdown();
  }

  void run()
  {
    const auto experiment_start = perf_clock::now();

    m_thread_pool.reserve(m_pubs.size() + m_subs.size());
    m_running = true;

    for (auto & sub : m_subs) {
      m_thread_pool.emplace_back(
        [&sub, this]() {
          while (m_running) {
            sub->run();
          }
        });
    }

    for (auto & pub : m_pubs) {
      m_thread_pool.emplace_back(
        [&pub, this]() {
          while (m_running) {
            pub->run();
          }
        });
    }

    perf_clock::time_point last_measurement_time{};

    while (!check_exit(experiment_start)) {
      const auto loop_start = perf_clock::now();
      const auto time_between_two_measurements =
        loop_start - last_measurement_time;

      // Collect measurements every second
      std::this_thread::sleep_for(std::chrono::seconds(1));

      for (auto & pub : m_pub_stats) {
        pub.update_stats(RunType::PUBLISHER, time_between_two_measurements);
      }
      for (auto & sub : m_sub_stats) {
        sub.update_stats(RunType::SUBSCRIBER, time_between_two_measurements);
      }

      if (ignore_first_seconds_of_experiment(experiment_start)) {
        auto results = std::make_shared<AnalysisResult>();
        results->m_experiment_start = loop_start - experiment_start;
        results->m_time_between_two_measurements =
          time_between_two_measurements;
        results->m_cpu_info = cpu_usage_tracker.get_cpu_usage();

        for (auto & pub : m_pub_stats) {
          pub.populate_stats(RunType::PUBLISHER, results);
        }
        for (auto & sub : m_sub_stats) {
          sub.populate_stats(RunType::SUBSCRIBER, results);
        }

        for (const auto & output : m_outputs) {
          output->update(results);
        }
      }
      last_measurement_time = loop_start;
    }
    m_running = false;
  }

private:
  bool ignore_first_seconds_of_experiment(
    const perf_clock::time_point & experiment_start)
  {
    const auto time_elapsed = perf_clock::now() - experiment_start;
    auto time_elapsed_s =
      std::chrono::duration_cast<std::chrono::seconds>(time_elapsed).count();

    if (time_elapsed_s > m_ec.rows_to_ignore()) {
      return true;
    } else {
      for (auto & pub : m_pub_stats) {
        pub.reset();
      }
      for (auto & sub : m_sub_stats) {
        sub.reset();
      }

      return false;
    }
  }

  bool check_exit(perf_clock::time_point experiment_start)
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
      std::chrono::duration<double>(perf_clock::now() - experiment_start)
      .count();

    if (runtime_sec > static_cast<double>(m_ec.max_runtime())) {
      std::cout << "Maximum runtime reached. Exiting." << std::endl;
      return true;
    } else {
      return false;
    }
  }
  const ExperimentConfiguration & m_ec;
  std::vector<std::shared_ptr<Output>> m_outputs;
  std::vector<DataStats> m_pub_stats;
  std::vector<DataStats> m_sub_stats;
  std::vector<std::thread> m_thread_pool;
  CPUsageTracker cpu_usage_tracker;
  std::atomic<bool> m_running{false};
  std::vector<std::shared_ptr<EntityType>> m_pubs;
  std::vector<std::shared_ptr<EntityType>> m_subs;
};

}  // namespace performance_test

#endif  // EXPERIMENT_EXECUTION__RUNNER_HPP_
