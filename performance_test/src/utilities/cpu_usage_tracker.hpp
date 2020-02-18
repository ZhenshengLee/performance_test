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

#include <boost/timer/timer.hpp>
#include <thread>

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
#include <odb/core.hxx>
#endif

namespace performance_test
{

struct CpuInfo
{
  CpuInfo(uint32_t cpu_cores, float cpu_usage)
  : m_cpu_cores(cpu_cores), m_cpu_usage(cpu_usage) {}

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  CpuInfo() {}
#endif

  uint32_t cpu_cores() const
  {
    return m_cpu_cores;
  }

  float cpu_usage() const
  {
    return m_cpu_usage;
  }

private:
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  friend class odb::access;
#endif
#pragma db default(0)
  uint32_t m_cpu_cores;
#pragma db default(0.0)
  float m_cpu_usage;
};

///  Calculate the CPU usage for the running experiment in the performance test
class CPUsageTracker
{
protected:
  boost::timer::cpu_timer m_cpu_timer;

public:
  CPUsageTracker()
  : m_cpu_timer() {}

  /**
    * \brief Computes the CPU usage % as (process_active_time/total_cpu_time)*100
    *
    */
  CpuInfo get_cpu_usage()
  {
    auto times = m_cpu_timer.elapsed();
    m_cpu_timer.start();

    auto cpu_cores = std::thread::hardware_concurrency();
    return CpuInfo(
      cpu_cores,
      100.0F * static_cast<float>(times.user + times.system) /
      static_cast<float>(times.wall * cpu_cores)
    );
  }
};
}  // namespace performance_test

#endif  // UTILITIES__CPU_USAGE_TRACKER_HPP_
