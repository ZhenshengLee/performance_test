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

#include "csv_output.hpp"

#include <iomanip>
#include <iostream>
#include <memory>

#include "../experiment_execution/analysis_result.hpp"

namespace performance_test
{
CsvOutput::CsvOutput()
: m_ec(ExperimentConfiguration::get()) {}

CsvOutput::~CsvOutput()
{
  close();
}

void CsvOutput::open()
{
  if (m_ec.is_setup() && !m_ec.logfile_name().empty()) {
    m_os.open(m_ec.logfile_name(), std::ofstream::out);
    m_is_open = true;

    std::cout << "Writing CSV output to: " << m_ec.logfile_name() << std::endl;

    // write experiment details
    m_os << m_ec;
    m_os << m_ec.get_external_info().m_to_log;
    m_os << std::endl << std::endl;

    // write header
    m_os << "---EXPERIMENT-START---" << std::endl;
    m_os << AnalysisResult::csv_header(true) << std::endl;
  }
}
void CsvOutput::update(std::shared_ptr<const AnalysisResult> result)
{
  m_os << result->to_csv_string(true) << std::endl;
}

void CsvOutput::close()
{
  if (m_is_open) {
    m_os.close();
  }
}

}  // namespace performance_test
