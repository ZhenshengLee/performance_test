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

#include "data_runner_factory.hpp"

#include <performance_test/generated_messages/messages.hpp>
#include <performance_test/for_each.hpp>

#include <string>
#include <memory>

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  #include "factories/rclcpp_ste_data_runner_factory.hpp"
  #include "factories/rclcpp_sste_data_runner_factory.hpp"
  #include "factories/rclcpp_waitset_data_runner_factory.hpp"
#endif

#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  #include "../communication_abstractions/fast_rtps_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  #include "../communication_abstractions/connext_dds_micro_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
#include "../communication_abstractions/connext_dds_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  #include "../communication_abstractions/cyclonedds_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
  #include "../communication_abstractions/iceoryx_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
  #include "../communication_abstractions/opendds_communicator.hpp"
#endif
#include "data_runner.hpp"

namespace performance_test
{

std::shared_ptr<DataRunnerBase> DataRunnerFactory::get(
  const std::string & msg_name,
  CommunicationMean com_mean,
  const RunType run_type)
{
  std::shared_ptr<DataRunnerBase> ptr;
  performance_test::for_each(
    messages::MessageTypeList(),
    [&ptr, msg_name, com_mean, run_type](const auto & msg_type) {
      using T = std::remove_cv_t<std::remove_reference_t<decltype(msg_type)>>;
      if (T::msg_name() == msg_name) {
        if (ptr) {
          throw std::runtime_error("It seems that two msgs have the same name");
        }
#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
        // These are moved into sub-factories to reduce the amount of template expansion
        // for this single file. If all three are handled in-place here, like the other
        // plugins, then sometimes the CI jobs will time out.
        if (com_mean == CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR) {
          ptr = RclcppSteDataRunnerFactory::get(msg_name, run_type);
        }
        if (com_mean == CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR) {
          ptr = RclcppSsteDataRunnerFactory::get(msg_name, run_type);
        }
        if (com_mean == CommunicationMean::RCLCPP_WAITSET) {
          ptr = RclcppWaitsetDataRunnerFactory::get(msg_name, run_type);
        }
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
        if (com_mean == CommunicationMean::FASTRTPS) {
          ptr = std::make_shared<DataRunner<FastRTPSCommunicator<T>>>(run_type);
        }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
        if (com_mean == CommunicationMean::CONNEXTDDSMICRO) {
          ptr = std::make_shared<DataRunner<RTIMicroDDSCommunicator<T>>>(run_type);
        }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
        if (com_mean == CommunicationMean::CONNEXTDDS) {
          ptr = std::make_shared<DataRunner<RTIDDSCommunicator<T>>>(run_type);
        }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
        if (com_mean == CommunicationMean::CYCLONEDDS) {
          ptr = std::make_shared<DataRunner<CycloneDDSCommunicator<T>>>(run_type);
        }
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
        if (com_mean == CommunicationMean::ICEORYX) {
          ptr = std::make_shared<DataRunner<IceoryxCommunicator<T>>>(run_type);
        }
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
        if (com_mean == CommunicationMean::OPENDDS) {
          ptr = std::make_shared<DataRunner<OpenDDSCommunicator<T>>>(run_type);
        }
#endif
      }
    });
  if (!ptr) {
    throw std::runtime_error(
            "A topic with the requested name does not exist or communication mean not supported.");
  }
  return ptr;
}

}  // namespace performance_test
