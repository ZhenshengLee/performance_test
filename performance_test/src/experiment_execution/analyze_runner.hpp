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

#ifndef EXPERIMENT_EXECUTION__ANALYZE_RUNNER_HPP_
#define EXPERIMENT_EXECUTION__ANALYZE_RUNNER_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "analysis_result.hpp"
#include "../data_running/data_runner_factory.hpp"
#include "../experiment_configuration/experiment_configuration.hpp"
#include "../outputs/output.hpp"
#include "../utilities/cpu_usage_tracker.hpp"

namespace performance_test
{
/**
 * \brief Sets up and runs an experiment.
 */
class AnalyzeRunner
{
public:
  /**
   * \brief Creates publisher and subscriber runners used for the expermiment.
   */
  AnalyzeRunner();

  /**
   * \brief Bind outputs to receive the experiment results.
   */
  void bind_output(std::shared_ptr<Output> output);

  /**
   * \brief Runs the experiment.
   */
  void run();

private:
  /**
   * \brief Analyzes and logs the state of the experiment.
   * \param loop_diff_start
   * \param experiment_diff_start
   * \return The analysis result
   */

  std::shared_ptr<const AnalysisResult> analyze(
    const std::chrono::nanoseconds loop_diff_start,
    const std::chrono::nanoseconds experiment_diff_start);

  /**
   * \brief Checks if the experiment is finished.
   * \param experiment_start The start of the experiment.
   * \return Is the experiment finnished
   */
  bool check_exit(std::chrono::steady_clock::time_point experiment_start) const;

  const ExperimentConfiguration & m_ec;
  std::vector<std::shared_ptr<Output>> m_outputs;
  std::vector<std::shared_ptr<DataRunnerBase>> m_pub_runners;
  std::vector<std::shared_ptr<DataRunnerBase>> m_sub_runners;
  mutable bool m_is_first_entry;
  CPUsageTracker cpu_usage_tracker;
};

}  // namespace performance_test

#endif  // EXPERIMENT_EXECUTION__ANALYZE_RUNNER_HPP_
