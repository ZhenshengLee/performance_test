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

#include "data_entity_factory.hpp"

#include <performance_test/for_each.hpp>
#include <performance_test/generated_messages/messages.hpp>

#include <memory>
#include <string>

#include "../communication_abstractions/apex_os_communicator.hpp"

namespace performance_test {

class ApexOsEntityFactory {
public:
  static std::shared_ptr<ApexOsEntity> get(const std::string &msg_name,
                                           const RunType run_type,
                                           DataStats &stats,
                                           const ExperimentConfiguration &ec) {
    std::shared_ptr<ApexOsEntity> ptr;
    performance_test::for_each(
        messages::MessageTypeList(),
        [&ptr, msg_name, run_type, &stats, &ec](const auto &msg_type) {
          using T =
              std::remove_cv_t<std::remove_reference_t<decltype(msg_type)>>;
          if (T::msg_name() == msg_name) {
            if (ptr) {
              throw std::runtime_error(
                  "It seems that two msgs have the same name");
            }

            switch (run_type) {
            case RunType::PUBLISHER:
              ptr = std::make_shared<ApexOsPublisher<typename T::RosType>>(
                  stats, ec);
              break;
            case RunType::SUBSCRIBER:
              ptr = std::make_shared<ApexOsSubscriber<typename T::RosType>>(
                  stats, ec);
              break;
            }
          }
        });
    if (!ptr) {
      throw std::runtime_error("A topic with the requested name does not exist "
                               "or communication mean not supported.");
    }
    return ptr;
  }
};
} // namespace performance_test
