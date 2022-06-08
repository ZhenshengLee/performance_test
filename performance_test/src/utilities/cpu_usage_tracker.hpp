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

#ifndef UTILITIES__CPU_USAGE_TRACKER_HPP_
#define UTILITIES__CPU_USAGE_TRACKER_HPP_

#if !defined(WIN32)
#include <sys/times.h>
#include <unistd.h>
#endif  // !defined(WIN32)
#include <thread>

#if defined(QNX)
#include <sys/neutrino.h>
#include <sys/syspage.h>
#include <sys/types.h>
#include <chrono>
#endif  // defined(QNX)

#include "../utilities/perf_clock.hpp"

namespace performance_test
{

struct CpuInfo
{
  CpuInfo(uint32_t cpu_cores, float cpu_usage)
  : m_cpu_cores(cpu_cores), m_cpu_usage(cpu_usage) {}

  uint32_t cpu_cores() const
  {
    return m_cpu_cores;
  }

  float cpu_usage() const
  {
    return m_cpu_usage;
  }

private:
  uint32_t m_cpu_cores;
  float m_cpu_usage;
};

///  Calculate the CPU usage for the running experiment in the performance test
class CPUsageTracker
{
public:
  CPUsageTracker()
  {
#if defined(QNX)
    m_tot_cpu_cores = _syspage_ptr->num_cpu;
#else
    m_cpu_cores = std::thread::hardware_concurrency();
#if !defined(WIN32)
    m_ticks_to_ns = 1000000000LL / ::sysconf(_SC_CLK_TCK);
#endif  // !defined(WIN32)
#endif
  }

  /**
    * \brief Computes the CPU usage % as (process_active_time/total_cpu_time)*100
    *
    */
  CpuInfo get_cpu_usage()
  {
#if defined(QNX)
    struct timespec cur_usage_st;
    float_t cpu_usage_local{};

    int64_t cur_time_ms =
      std::chrono::time_point_cast<std::chrono::milliseconds>(perf_clock::now()).
      time_since_epoch().count();

    if (-1 != clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cur_usage_st)) {
      int64_t cur_active_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::seconds{cur_usage_st.tv_sec} +
        std::chrono::nanoseconds{cur_usage_st.tv_nsec}).count();

      int64_t active_time_ms = (cur_active_time - m_prev_active_time);

      int64_t total_time_ms = ((cur_time_ms - m_prev_total_time) *
        static_cast<int64_t>(m_tot_cpu_cores));

      cpu_usage_local = (static_cast<float_t>(active_time_ms) /
        static_cast<float_t>(total_time_ms)) * 100.0F;

      m_prev_active_time = cur_active_time;
      m_prev_total_time = cur_time_ms;

    } else {
      throw std::runtime_error("get_cpu_usage: Error getting process CPU usage time");
    }

    CpuInfo cpu_info_local{m_tot_cpu_cores, cpu_usage_local};
    return cpu_info_local;
#else
    auto wall_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
#if defined(WIN32)
    auto system_time = m_system_time;
    auto user_time = m_user_time;
#else
    tms tm;
    ::times(&tm);
    auto system_time = (tm.tms_stime + tm.tms_cstime) * m_ticks_to_ns;
    auto user_time = (tm.tms_utime + tm.tms_cutime) * m_ticks_to_ns;
#endif
    auto wall_diff = wall_time - m_wall_time;
    auto system_diff = system_time - m_system_time;
    auto user_diff = user_time - m_user_time;

    m_wall_time = wall_time;
    m_system_time = system_time;
    m_user_time = user_time;

    return CpuInfo(
      m_cpu_cores,
      100.0F * static_cast<float>(user_diff + system_diff) /
      static_cast<float>(wall_diff * m_cpu_cores)
    );
#endif  // defined(QNX)
  }

private:
#if defined(QNX)
  int64_t m_prev_active_time{};
  int64_t m_prev_total_time{};
  uint32_t m_tot_cpu_cores{};
#else
  unsigned int m_cpu_cores{};
  int64_t m_ticks_to_ns{};
  int64_t m_user_time{};
  int64_t m_system_time{};
  int64_t m_wall_time{};
#endif  // defined(QNX)
};
}  // namespace performance_test

#endif  // UTILITIES__CPU_USAGE_TRACKER_HPP_
