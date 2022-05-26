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

#include "communication_mean.hpp"

#include <string>

namespace performance_test
{

std::string to_string(const CommunicationMean cm)
{
#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
  if (cm == CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR) {
    return "RCLCPP_SINGLE_THREADED_EXECUTOR";
  }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
  if (cm == CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR) {
    return "RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR";
  }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
  if (cm == CommunicationMean::RCLCPP_WAITSET) {
    return "RCLCPP_WAITSET";
  }
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  if (cm == CommunicationMean::ApexOSPollingSubscription) {
    return "ApexOSPollingSubscription";
  }
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  if (cm == CommunicationMean::FASTRTPS) {
    return "FASTRTPS";
  }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  if (cm == CommunicationMean::CONNEXTDDSMICRO) {
    return "CONNEXTDDSMICRO";
  }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
  if (cm == CommunicationMean::CONNEXTDDS) {
    return "CONNEXTDDS";
  }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  if (cm == CommunicationMean::CYCLONEDDS) {
    return "CYCLONEDDS";
  }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
  if (cm == CommunicationMean::CYCLONEDDS_CXX) {
    return "CYCLONEDDS_CXX";
  }
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
  if (cm == CommunicationMean::ICEORYX) {
    return "ICEORYX";
  }
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
  if (cm == CommunicationMean::OPENDDS) {
    return "OpenDDS";
  }
#endif
  throw std::invalid_argument("Enum value not supported!");
}

}  // namespace performance_test
