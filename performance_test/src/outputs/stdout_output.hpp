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

#ifndef OUTPUTS__STDOUT_OUTPUT_HPP_
#define OUTPUTS__STDOUT_OUTPUT_HPP_

#include <iostream>
#include <memory>

#include "../experiment_configuration/experiment_configuration.hpp"
#include "output.hpp"

namespace performance_test
{

class StdoutOutput : public Output
{
public:
  StdoutOutput();
  virtual ~StdoutOutput() = default;

  void open() override;
  void update(std::shared_ptr<const AnalysisResult> result) override;
  void close() override;

private:
  const ExperimentConfiguration & m_ec;
  bool m_refresh = false;
};

}  // namespace performance_test

#endif  // OUTPUTS__STDOUT_OUTPUT_HPP_
