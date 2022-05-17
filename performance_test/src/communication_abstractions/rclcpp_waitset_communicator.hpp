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

#ifndef COMMUNICATION_ABSTRACTIONS__RCLCPP_WAITSET_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__RCLCPP_WAITSET_COMMUNICATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>

#include "../experiment_configuration/qos_abstraction.hpp"

#include "rclcpp_communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{

/// Communication plugin for ROS 2 using waitsets for the subscription side.
template<class Msg>
class RclcppWaitsetCommunicator : public RclcppCommunicator<Msg>
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename RclcppCommunicator<Msg>::DataType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit RclcppWaitsetCommunicator(DataStats & stats)
  : RclcppCommunicator<Msg>(stats),
    m_subscription(nullptr)
  {
    auto hz = static_cast<double>(this->m_ec.rate());
    auto period = std::chrono::duration<double>(1.0 / hz);
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
    this->m_timeout = std::max(10 * period_ns, std::chrono::nanoseconds(100 * 1000 * 1000));
  }

  void update_subscription() override
  {
    if (!m_subscription) {
      m_subscription = this->m_node->template create_subscription<DataType>(
        this->m_ec.topic_name() + this->m_ec.sub_topic_postfix(), this->m_ROS2QOSAdapter,
        [this](const typename DataType::SharedPtr data) {this->callback(data);});
      m_waitset = std::make_unique<rclcpp::WaitSet>();
      m_waitset->add_subscription(m_subscription);
    }

    // This timeout ensures the wait will unblock when the program receives an INT signal
    const auto wait_ret = m_waitset->wait(m_timeout);

    if (wait_ret.kind() == rclcpp::Ready) {
      DataType msg;
      rclcpp::MessageInfo msg_info;
      bool success = m_subscription->take(msg, msg_info);
      if (success) {
        this->callback(msg);
      }
    }
  }

private:
  std::shared_ptr<::rclcpp::Subscription<DataType>> m_subscription;
  std::unique_ptr<rclcpp::WaitSet> m_waitset;
  std::chrono::nanoseconds m_timeout;
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__RCLCPP_WAITSET_COMMUNICATOR_HPP_
