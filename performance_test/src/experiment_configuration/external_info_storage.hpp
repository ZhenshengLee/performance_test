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

#ifndef EXPERIMENT_CONFIGURATION__EXTERNAL_INFO_STORAGE_HPP_
#define EXPERIMENT_CONFIGURATION__EXTERNAL_INFO_STORAGE_HPP_

#include <iostream>
#include <string>

namespace performance_test
{

class ExternalInfoStorage
{
public:
  /**
 * \brief Constructor that reads values from environment variables.
 */
  ExternalInfoStorage();

  std::string m_to_log;
  std::string m_githash;
  std::string m_platform;
  std::string m_branch;
  std::string m_architecture;
  std::string m_ci;
};
}  // namespace performance_test
#endif  // EXPERIMENT_CONFIGURATION__EXTERNAL_INFO_STORAGE_HPP_
