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

#include "json_output.hpp"

#include <iomanip>
#include <iostream>
#include <memory>

#include "../experiment_execution/analysis_result.hpp"
#include "../utilities/json_logger.hpp"

namespace performance_test
{
JsonOutput::JsonOutput()
: m_ec(ExperimentConfiguration::get()) {}

JsonOutput::~JsonOutput()
{
  close();
}

void JsonOutput::open()
{
  if (m_ec.is_setup() && !m_ec.logfile_name().empty()) {
    m_os.open(m_ec.logfile_name(), std::ofstream::out);
    m_is_open = true;

    std::cout << "Writing JSON output to: " << m_ec.logfile_name() << std::endl;
  }
}
void JsonOutput::update(std::shared_ptr<const AnalysisResult> result)
{
  m_results.push_back(result);
}

void JsonOutput::close()
{
  if (m_is_open) {
    JsonLogger::log(m_ec, m_results, m_os);
    m_os.close();
  }
}

}  // namespace performance_test
