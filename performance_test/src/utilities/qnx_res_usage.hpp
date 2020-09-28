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

#ifndef UTILITIES__QNX_RES_USAGE_HPP_
#define UTILITIES__QNX_RES_USAGE_HPP_

#if defined(QNX)

#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>

#include <fstream>
#include <string>

namespace performance_test
{
namespace qnx_res
{

///
/// \brief Get process physical(RSS) memory usage
/// \return memory usage size on success, throw an exception on error
uint64_t get_proc_rss_mem()
{
  std::string line;

  pid_t my_pid = getpid();
  std::ifstream vmstat_file("/proc/" + std::to_string(my_pid) + "/vmstat", std::ifstream::in);
  const std::string rss_string("as_stats.rss_wired");
  const std::size_t rss_str_len = rss_string.size();

  while (std::getline(vmstat_file, line)) {
    if (!line.compare(0, rss_str_len, rss_string)) {
      auto first_del = line.find("=");
      auto sec_del = line.find(" ");
      auto value = line.substr(first_del + 1, sec_del - first_del);
      uint64_t rss_sz = std::stoul(value, 0, 16);
      rss_sz = (rss_sz * static_cast<uint64_t>(getpagesize()));
      return rss_sz;
    }
  }
  return 0u;
}
}  // namespace qnx_res

}  // namespace performance_test

#endif  // defined(QNX)
#endif  // UTILITIES__QNX_RES_USAGE_HPP_
