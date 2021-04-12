// Copyright 2017-2021 Apex.AI, Inc.
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

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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
  std::ifstream ifile("/proc/" + std::to_string(my_pid) + "/pmap");
  int num_line = 0;
  uint64_t total_memory = 0u;

  while (std::getline(ifile, line)) {
    // Discard first line, contains only the header
    if (num_line > 0) {
      std::istringstream iss{line};  // Construct string stream from line
      std::vector<std::string> fields;
      std::string field;
      while (std::getline(iss, field, ',')) {
        fields.push_back(field);
      }
      // Convert field 9(Rsv) to check memory usage
      total_memory += std::stoul(fields[8], nullptr, 16);
    }
    num_line++;
  }
  return total_memory;
}
}  // namespace qnx_res

}  // namespace performance_test

#endif  // defined(QNX)
#endif  // UTILITIES__QNX_RES_USAGE_HPP_
