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

#include <memory>
#include <string>

#include "data_entity_factory.hpp"

#include <performance_test/for_each.hpp>
#include <performance_test/generated_messages/messages.hpp>


#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
#include "factories/rclcpp_ste_data_runner_factory.hpp"
#endif

#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
  #include "factories/rclcpp_sste_data_runner_factory.hpp"
#endif

#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
  #include "factories/rclcpp_waitset_data_runner_factory.hpp"
#endif

#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
#include "../communication_abstractions/apex_os_polling_subscription_communicator.hpp"
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

#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
#include "../communication_abstractions/cyclonedds_cxx_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
#include "../communication_abstractions/iceoryx_communicator.hpp"
#endif

#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
#include "../communication_abstractions/opendds_communicator.hpp"
#endif
#include "data_publisher.hpp"
#include "data_subscriber.hpp"

namespace performance_test
{

std::shared_ptr<DataEntity> DataEntityFactory::get(
  const std::string & msg_name,
  CommunicationMean com_mean,
  const RunType run_type,
  DataStats & stats)
{
  std::shared_ptr<DataEntity> ptr;
  performance_test::for_each(
    messages::MessageTypeList(),
    [&ptr, msg_name, com_mean, run_type, &stats](const auto & msg_type) {
      using T = std::remove_cv_t<std::remove_reference_t<decltype(msg_type)>>;
      if (T::msg_name() == msg_name) {
        if (ptr) {
          throw std::runtime_error(
            "It seems that two msgs have the same name");
        }
#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
        // These are moved into sub-factories to reduce the amount of template expansion
        // for this single file. If all three are handled in-place here, like the other
        // plugins, then sometimes the CI jobs will time out.
        if (com_mean == CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR) {
          ptr = RclcppSteDataRunnerFactory::get(msg_name, run_type, stats);
        }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
        // These are moved into sub-factories to reduce the amount of template expansion
        // for this single file. If all three are handled in-place here, like the other
        // plugins, then sometimes the CI jobs will time out.
        if (com_mean == CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR) {
          ptr = RclcppSsteDataRunnerFactory::get(msg_name, run_type, stats);
        }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
        // These are moved into sub-factories to reduce the amount of template expansion
        // for this single file. If all three are handled in-place here, like the other
        // plugins, then sometimes the CI jobs will time out.
        if (com_mean == CommunicationMean::RCLCPP_WAITSET) {
          ptr = RclcppWaitsetDataRunnerFactory::get(msg_name, run_type, stats);
        }
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
        if (com_mean == CommunicationMean::ApexOSPollingSubscription) {
          switch (run_type) {
            case RunType::PUBLISHER:
              ptr = std::make_shared<
                DataPublisher<ApexOSPollingSubscriptionCommunicator<T>>>(stats);
              break;
            case RunType::SUBSCRIBER:
              ptr = std::make_shared<
                DataSubscriber<ApexOSPollingSubscriptionCommunicator<T>>>(stats);
              break;
          }
        }
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
        if (com_mean == CommunicationMean::FASTRTPS) {
          switch (run_type) {
            case RunType::PUBLISHER:
              ptr = std::make_shared<DataPublisher<FastRTPSCommunicator<T>>>(
                stats);
              break;
            case RunType::SUBSCRIBER:
              ptr = std::make_shared<DataSubscriber<FastRTPSCommunicator<T>>>(
                stats);
              break;
          }
        }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
        if (com_mean == CommunicationMean::CONNEXTDDSMICRO) {
          switch (run_type) {
            case RunType::PUBLISHER:
              ptr = std::make_shared<DataPublisher<RTIMicroDDSCommunicator<T>>>(
                stats);
              break;
            case RunType::SUBSCRIBER:
              ptr =
              std::make_shared<DataSubscriber<RTIMicroDDSCommunicator<T>>>(
                stats);
              break;
          }
        }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
        if (com_mean == CommunicationMean::CONNEXTDDS) {
          ptr = std::make_shared<DataPublisher<RTIDDSCommunicator<T>>>(stats);
        }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
        if (com_mean == CommunicationMean::CYCLONEDDS) {
          switch (run_type) {
            case RunType::PUBLISHER:
              ptr = std::make_shared<DataPublisher<CycloneDDSCommunicator<T>>>(
                stats);
              break;
            case RunType::SUBSCRIBER:
              ptr = std::make_shared<DataSubscriber<CycloneDDSCommunicator<T>>>(
                stats);
              break;
          }
        }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
        if (com_mean == CommunicationMean::CYCLONEDDS_CXX) {
          switch (run_type) {
            case RunType::PUBLISHER:
              ptr =
              std::make_shared<DataPublisher<CycloneDDSCXXCommunicator<T>>>(
                stats);
              break;
            case RunType::SUBSCRIBER:
              ptr = std::make_shared<
                DataSubscriber<CycloneDDSCXXCommunicator<T>>>(stats);
              break;
          }
        }
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
        if (com_mean == CommunicationMean::ICEORYX) {
          switch (run_type) {
            case RunType::PUBLISHER:
              ptr = std::make_shared<DataPublisher<IceoryxCommunicator<T>>>(
                stats);
              break;
            case RunType::SUBSCRIBER:
              ptr = std::make_shared<DataSubscriber<IceoryxCommunicator<T>>>(
                stats);
              break;
          }
        }
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
        if (com_mean == CommunicationMean::OPENDDS) {
          switch (run_type) {
            case RunType::PUBLISHER:
              ptr = std::make_shared<DataPublisher<OpenDDSCommunicator<T>>>(
                stats);
              break;
            case RunType::SUBSCRIBER:
              ptr = std::make_shared<DataSubscriber<OpenDDSCommunicator<T>>>(
                stats);
              break;
          }
        }
#endif
      }
    });
  if (!ptr) {
    throw std::runtime_error(
            "A topic with the requested name does not exist "
            "or communication mean not supported.");
  }
  return ptr;
}

}  // namespace performance_test
