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

#include <string>
#include "external_info_storage.hpp"

namespace performance_test
{

ExternalInfoStorage::ExternalInfoStorage()
{
  const auto ptr = std::getenv("APEX_PERFORMANCE_TEST");
  if (ptr) {
    std::stringstream ss;
    ss.str(ptr);
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    for (boost::property_tree::ptree::iterator iter = pt.begin(); iter != pt.end(); iter++) {
      m_to_log += iter->first + ": " + iter->second.data() + "\n";
    }
  #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
    save_to_db(pt);
  #endif
  }
}

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
void ExternalInfoStorage::save_to_db(boost::property_tree::ptree pt)
{
  try {
    m_githash = pt.get<std::string>("Githash");
  } catch (boost::property_tree::ptree_error & e) {
    std::cout << e.what() << std::endl;
  }
  try {
    m_platform = pt.get<std::string>("Platform");
  } catch (boost::property_tree::ptree_error & e) {
    std::cout << e.what() << std::endl;
  }
  try {
    m_branch = pt.get<std::string>("Branch");
  } catch (boost::property_tree::ptree_error & e) {
    std::cout << e.what() << std::endl;
  }
  try {
    m_architecture = pt.get<std::string>("Architecture");
  } catch (boost::property_tree::ptree_error & e) {
    std::cout << e.what() << std::endl;
  }
  try {
    m_ci = pt.get<std::string>("CI");
  } catch (boost::property_tree::ptree_error & e) {
    std::cout << e.what() << std::endl;
  }
}
#endif

}  // namespace performance_test
