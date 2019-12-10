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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <iostream>
#include <string>

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #include <odb/core.hxx>
#endif

namespace performance_test
{

class ExternalInfoStorage
{
public:
  /**
 * \brief Constructor that reads values from environment variables.
 */
  ExternalInfoStorage();

  #pragma db transient
  std::string m_to_log;

private:
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  friend class odb::access;

  /**
  * \brief Assigns values to the provided member variables if they exist in environment variable.
  * \param pt property tree with the values extracted from environment variable
  */
  void save_to_db(boost::property_tree::ptree pt);

  #pragma db default()
  std::string m_githash;
  #pragma db default()
  std::string m_platform;
  #pragma db default()
  std::string m_branch;
  #pragma db default()
  std::string m_architecture;
  #pragma db default()
  std::string m_ci;
#endif
};
}  // namespace performance_test
#endif  // EXPERIMENT_CONFIGURATION__EXTERNAL_INFO_STORAGE_HPP_
