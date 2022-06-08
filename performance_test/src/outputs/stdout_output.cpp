// Copyright 2021 Apex.AI, Inc.
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

#include <string>
#include <chrono>
#include <iostream>
#include <sstream>
#include <memory>

#include <tabulate/table.hpp>

#include "stdout_output.hpp"
#include "../experiment_configuration/experiment_configuration.hpp"
#include "../experiment_execution/analysis_result.hpp"

namespace performance_test
{

StdoutOutput::StdoutOutput()
: m_ec(ExperimentConfiguration::get()) {}

void StdoutOutput::open()
{
  // write experiment details
  std::cout << m_ec;
  std::cout << m_ec.get_external_info().m_to_log;
  std::cout << std::endl << std::endl;
}

void StdoutOutput::update(std::shared_ptr<const AnalysisResult> result)
{
  if (result) {
    // clear old table
    if (m_refresh) {
      for (int i = 0; i < 37; i++) {
        std::cout << "\033[F";
      }
    }

    // construct tables with current results
    tabulate::Table sample_table;
    sample_table.add_row(
      {"recv", "sent", "lost", "data_recv", "relative_loss"});
    sample_table.add_row(
      {std::to_string(result->m_num_samples_received),
        std::to_string(result->m_num_samples_sent),
        std::to_string(result->m_num_samples_lost),
        std::to_string(result->m_total_data_received),
        std::to_string(
          static_cast<double>(result->m_num_samples_lost) /
          static_cast<double>(result->m_num_samples_sent)
        )});

    tabulate::Table latency_table;
    latency_table.add_row({"min", "max", "mean", "variance"});
    if (result->m_latency.n() > 0) {
      latency_table.add_row(
        {std::to_string(result->m_latency.min()),
          std::to_string(result->m_latency.max()),
          std::to_string(result->m_latency.mean()),
          std::to_string(result->m_latency.variance())});
    } else {
      latency_table.add_row({"-", "-", "-", "-"});
    }


    tabulate::Table system_statistics_table;
#if !defined(WIN32)
    std::chrono::nanoseconds utime_ns =
      std::chrono::seconds(result->m_sys_usage.ru_utime.tv_sec) +
      std::chrono::microseconds(result->m_sys_usage.ru_utime.tv_usec);
    std::chrono::nanoseconds stime_ns =
      std::chrono::seconds(result->m_sys_usage.ru_stime.tv_sec) +
      std::chrono::microseconds(result->m_sys_usage.ru_stime.tv_usec);

    system_statistics_table.add_row(
      {"utime", "stime", "maxrss", "ixrss", "idrss", "isrss", "minflt", "majflt"});
    system_statistics_table.add_row(
      {std::to_string(utime_ns.count()),
        std::to_string(stime_ns.count()),
        std::to_string(result->m_sys_usage.ru_maxrss),
        std::to_string(result->m_sys_usage.ru_ixrss),
        std::to_string(result->m_sys_usage.ru_idrss),
        std::to_string(result->m_sys_usage.ru_isrss),
        std::to_string(result->m_sys_usage.ru_minflt),
        std::to_string(result->m_sys_usage.ru_majflt)});
    system_statistics_table.add_row(
      {"nswap", "inblock", "oublock", "msgsnd",
        "msgrcv", "nsignals", "nvcsw", "nivcsw"});
    system_statistics_table.add_row(
      {std::to_string(result->m_sys_usage.ru_nswap),
        std::to_string(result->m_sys_usage.ru_inblock),
        std::to_string(result->m_sys_usage.ru_oublock),
        std::to_string(result->m_sys_usage.ru_msgsnd),
        std::to_string(result->m_sys_usage.ru_msgrcv),
        std::to_string(result->m_sys_usage.ru_nsignals),
        std::to_string(result->m_sys_usage.ru_nvcsw),
        std::to_string(result->m_sys_usage.ru_nivcsw)});
#endif

    tabulate::Table experiment_time_table;
    experiment_time_table.add_row(
      {"T_experiment",
        std::to_string(
          std::chrono::duration_cast<std::chrono::duration<float>>(
            result->m_experiment_start)
          .count()),
        "T_loop",
        std::to_string(
          std::chrono::duration_cast<std::chrono::duration<float>>(
            result->m_time_between_two_measurements)
          .count())});

    // create layout (tables without borders)
    tabulate::Table timing_table;
    timing_table.add_row({"run time"});
    timing_table.add_row({experiment_time_table});

    timing_table.format()
    .border_top(" ")
    .border_bottom(" ")
    .border_left(" ")
    .border_right(" ")
    .corner(" ");

    tabulate::Table packets_table;
    packets_table.add_row({"samples", "latency"});
    packets_table.add_row({sample_table, latency_table});

    packets_table.format()
    .border_top(" ")
    .border_bottom(" ")
    .border_left(" ")
    .border_right(" ")
    .corner(" ");

    tabulate::Table system_usage_table;
    system_usage_table.add_row({"system usage"});
    system_usage_table.add_row({system_statistics_table});

    system_usage_table.format()
    .border_top(" ")
    .border_bottom(" ")
    .border_left(" ")
    .border_right(" ")
    .corner(" ");

    // print table
    std::cout << timing_table << std::endl;
    std::cout << packets_table << std::endl;
    std::cout << system_usage_table << std::endl;

    // flag to refresh table on next update
    if (!m_refresh) {
      m_refresh = true;
    }
  }
}

void StdoutOutput::close() {}

}  // namespace performance_test
