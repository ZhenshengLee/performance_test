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

#ifndef EXPERIMENT_CONFIGURATION__COMMUNICATION_MEAN_HPP_
#define EXPERIMENT_CONFIGURATION__COMMUNICATION_MEAN_HPP_

#include <string>
#include <ostream>

namespace performance_test
{

/**
 * \brief Available means of communication.
 */
enum class CommunicationMean
{
#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
  RCLCPP_SINGLE_THREADED_EXECUTOR,
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
  RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR,
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
  RCLCPP_WAITSET,
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  ApexOSPollingSubscription,
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
  FASTRTPS,
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
  CONNEXTDDSMICRO,
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
  CONNEXTDDS,
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
  CYCLONEDDS,
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
  CYCLONEDDS_CXX,
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
  ICEORYX,
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
  OPENDDS,
#endif
  INVALID
};

std::string to_string(const CommunicationMean cm);

/// Outstream operator for CommunicationMean.
inline std::ostream & operator<<(std::ostream & stream, const CommunicationMean cm)
{
  return stream << to_string(cm);
}

}  // namespace performance_test

#endif  // EXPERIMENT_CONFIGURATION__COMMUNICATION_MEAN_HPP_
