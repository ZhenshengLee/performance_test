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

#include "rclcpp_waitset_data_runner_factory.hpp"

#include <string>
#include <memory>

#include <performance_test/generated_messages/messages.hpp>
#include <performance_test/for_each.hpp>

#include "../data_publisher.hpp"
#include "../data_subscriber.hpp"
#include "../../communication_abstractions/rclcpp_waitset_communicator.hpp"

namespace performance_test
{
namespace RclcppWaitsetDataRunnerFactory
{

std::shared_ptr<DataEntity> get(
  const std::string & msg_name,
  const RunType run_type,
  DataStats & stats)
{
  std::shared_ptr<DataEntity> ptr;
  performance_test::for_each(
    messages::MessageTypeList(),
    [&ptr, msg_name, run_type, &stats](const auto & msg_type) {
      using T = std::remove_cv_t<std::remove_reference_t<decltype(msg_type)>>;
      if (T::msg_name() == msg_name) {
        switch (run_type) {
          case RunType::PUBLISHER:
            ptr = std::make_shared<
              DataPublisher<RclcppWaitsetCommunicator<T>>>(
              stats);
            break;
          case RunType::SUBSCRIBER:
            ptr = std::make_shared<
              DataSubscriber<RclcppWaitsetCommunicator<T>>>(
              stats);
            break;
        }
      }
    });
  if (!ptr) {
    throw std::runtime_error(
            "A topic with the requested name does not exist or communication mean not supported.");
  }
  return ptr;
}
}  // namespace RclcppWaitsetDataRunnerFactory
}  // namespace performance_test
