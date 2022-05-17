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

#ifndef COMMUNICATION_ABSTRACTIONS__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__RCLCPP_CALLBACK_COMMUNICATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <atomic>

#include "rclcpp_communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{
/// Communication plugin for ROS 2 using ROS 2 callbacks for the subscription side.
template<class Msg, class Executor>
class RclcppCallbackCommunicator : public RclcppCommunicator<Msg>
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename RclcppCommunicator<Msg>::DataType;

  explicit RclcppCallbackCommunicator(DataStats & stats)
  : RclcppCommunicator<Msg>(stats), m_subscription(nullptr)
  {
    m_executor.add_node(this->m_node);
  }

  void update_subscription() override
  {
    if (!m_subscription) {
      m_subscription = this->m_node->template create_subscription<DataType>(
        this->m_ec.topic_name() + this->m_ec.sub_topic_postfix(), this->m_ROS2QOSAdapter,
        [this](const typename DataType::SharedPtr data) {this->callback(data);});
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
      if (this->m_ec.expected_num_pubs() > 0) {
        m_subscription->wait_for_matched(
          this->m_ec.expected_num_pubs(),
          this->m_ec.expected_wait_for_matched_timeout());
      }
#endif
    }
    m_executor.spin_once(std::chrono::milliseconds(100));
  }

private:
  Executor m_executor;
  std::shared_ptr<::rclcpp::Subscription<DataType>> m_subscription;
};

#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
template<class Msg>
using RclcppSingleThreadedExecutorCommunicator =
  RclcppCallbackCommunicator<Msg, rclcpp::executors::SingleThreadedExecutor>;
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
template<class Msg>
using RclcppStaticSingleThreadedExecutorCommunicator =
  RclcppCallbackCommunicator<Msg, rclcpp::executors::StaticSingleThreadedExecutor>;
#endif
}  // namespace performance_test
#endif  // COMMUNICATION_ABSTRACTIONS__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
