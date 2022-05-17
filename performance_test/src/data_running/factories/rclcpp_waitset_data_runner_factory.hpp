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

#ifndef DATA_RUNNING__FACTORIES__RCLCPP_WAITSET_DATA_RUNNER_FACTORY_HPP_
#define DATA_RUNNING__FACTORIES__RCLCPP_WAITSET_DATA_RUNNER_FACTORY_HPP_

#include <memory>
#include <string>

#include "../data_entity.hpp"

namespace performance_test
{
namespace RclcppWaitsetDataRunnerFactory
{
std::shared_ptr<DataEntity> get(
  const std::string & msg_name,
  const RunType run_type,
  DataStats & stats);
}  // namespace RclcppWaitsetDataRunnerFactory
}  // namespace performance_test

#endif  // DATA_RUNNING__FACTORIES__RCLCPP_WAITSET_DATA_RUNNER_FACTORY_HPP_
