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

#include "analysis_result.hpp"

#if !defined(WIN32)
#include <sys/times.h>
#endif  // !defined(WIN32)

#include <iomanip>
#include <string>

#include "../utilities/qnx_res_usage.hpp"

namespace performance_test
{

#if !defined(WIN32)
std::ostream & operator<<(std::ostream & stream, const timeval & e)
{
  return stream << double(e.tv_sec) + double(e.tv_usec) / 1000000.0;
}
#endif  // !defined(WIN32)

AnalysisResult::AnalysisResult()
: m_cpu_info([]() {
      CPUsageTracker cpu_usage_tracker;
      return cpu_usage_tracker.get_cpu_usage();
    } ()) {
#if !defined(WIN32)
  const auto ret = getrusage(RUSAGE_SELF, &m_sys_usage);
#if defined(QNX)
  // QNX getrusage() max_rss does not give the correct value. Using a
  // different method to get the RSS value and converting into KBytes
  m_sys_usage.ru_maxrss =
    (static_cast<int64_t>(performance_test::qnx_res::get_proc_rss_mem()) /
    1024);
#endif
  if (ret != 0) {
    throw std::runtime_error("Could not get system resource usage.");
  }
#endif  // !defined(WIN32)
  if (m_num_samples_received != static_cast<uint64_t>(m_latency.n())) {
    // TODO(andreas.pasternak): Commented out flaky assertion. Need to check
    // if it actually a bug.
    /*throw std::runtime_error("Statistics result sample size does not
       match: "
                             + std::to_string(m_num_samples_received) + " /
       "
                             + std::to_string(m_latency.n()));*/
  }
}

std::string AnalysisResult::csv_header(const bool pretty_print, std::string st)
{
  if (pretty_print) {
    st += "\t";
  }

  std::stringstream ss;
  ss << "T_experiment" << st;
  ss << "T_loop" << st;
  ss << "received" << st;
  ss << "sent" << st;
  ss << "lost" << st;
  ss << "relative_loss" << st;

  ss << "data_received" << st;

  ss << "latency_min (ms)" << st;
  ss << "latency_max (ms)" << st;
  ss << "latency_mean (ms)" << st;
  ss << "latency_variance (ms)" << st;

#if !defined(WIN32)
  ss << "ru_utime" << st;
  ss << "ru_stime" << st;
  ss << "ru_maxrss" << st;
  ss << "ru_ixrss" << st;
  ss << "ru_idrss" << st;
  ss << "ru_isrss" << st;
  ss << "ru_minflt" << st;
  ss << "ru_majflt" << st;
  ss << "ru_nswap" << st;
  ss << "ru_inblock" << st;
  ss << "ru_oublock" << st;
  ss << "ru_msgsnd" << st;
  ss << "ru_msgrcv" << st;
  ss << "ru_nsignals" << st;
  ss << "ru_nvcsw" << st;
  ss << "ru_nivcsw" << st;
#endif

  ss << "cpu_usage (%)";

  return ss.str();
}

std::string AnalysisResult::to_csv_string(const bool pretty_print, std::string st) const
{
  if (pretty_print) {
    st += "\t\t";
  }

  std::stringstream ss;

  ss << std::fixed;
  ss << std::chrono::duration_cast<std::chrono::duration<float>>(m_experiment_start).count() << st;
  ss << std::chrono::duration_cast<std::chrono::duration<float>>(
    m_time_between_two_measurements)
    .count() <<
    st;
  ss << std::setprecision(0);
  ss << m_num_samples_received << st;
  ss << m_num_samples_sent << st;
  ss << m_num_samples_lost << st;
  ss << std::setprecision(2);
  ss << static_cast<double>(m_num_samples_lost) / static_cast<double>(m_num_samples_sent) << st;

  ss << std::to_string(m_total_data_received) << st;

  ss << std::setprecision(4);
  ss << std::defaultfloat;

  ss << m_latency.min() * 1000.0 << st;
  ss << m_latency.max() * 1000.0 << st;
  ss << m_latency.mean() * 1000.0 << st;
  ss << m_latency.variance() * 1000.0 << st;

  /* See http://www.gnu.org/software/libc/manual/html_node/Resource-Usage.html
   * for a detailed explanation of the output below
   */

#if !defined(WIN32)
  ss << m_sys_usage.ru_utime << st;
  ss << m_sys_usage.ru_stime << st;
  ss << std::to_string(m_sys_usage.ru_maxrss) << st;
  ss << std::to_string(m_sys_usage.ru_ixrss) << st;
  ss << std::to_string(m_sys_usage.ru_idrss) << st;
  ss << std::to_string(m_sys_usage.ru_isrss) << st;
  ss << std::to_string(m_sys_usage.ru_minflt) << st;
  ss << std::to_string(m_sys_usage.ru_majflt) << st;
  ss << std::to_string(m_sys_usage.ru_nswap) << st;
  ss << std::to_string(m_sys_usage.ru_inblock) << st;
  ss << std::to_string(m_sys_usage.ru_oublock) << st;
  ss << std::to_string(m_sys_usage.ru_msgsnd) << st;
  ss << std::to_string(m_sys_usage.ru_msgrcv) << st;
  ss << std::to_string(m_sys_usage.ru_nsignals) << st;
  ss << std::to_string(m_sys_usage.ru_nvcsw) << st;
  ss << std::to_string(m_sys_usage.ru_nivcsw) << st;
#endif

  ss << m_cpu_info.cpu_usage();

  return ss.str();
}

}  // namespace performance_test
