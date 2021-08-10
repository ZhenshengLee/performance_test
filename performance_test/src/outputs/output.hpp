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

#ifndef OUTPUTS__OUTPUT_HPP_
#define OUTPUTS__OUTPUT_HPP_

#include <memory>

#include "../experiment_execution/analysis_result.hpp"

namespace performance_test
{

///
/// @brief Interface for outputting test data experiment
///
class Output
{
public:
  Output() = default;
  virtual ~Output() = default;

  /// @brief initialize the output
  virtual void open() = 0;

  /// @brief update output with given result
  virtual void update(std::shared_ptr<const performance_test::AnalysisResult> result) = 0;

  /// @brief close output cleanly
  virtual void close() = 0;
};

}  // namespace performance_test

#endif  // OUTPUTS__OUTPUT_HPP_
