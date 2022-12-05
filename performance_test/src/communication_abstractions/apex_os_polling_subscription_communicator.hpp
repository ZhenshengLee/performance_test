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

#ifndef COMMUNICATION_ABSTRACTIONS__APEX_OS_POLLING_SUBSCRIPTION_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__APEX_OS_POLLING_SUBSCRIPTION_COMMUNICATOR_HPP_

#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "../experiment_configuration/qos_abstraction.hpp"

#include "communicator.hpp"
#include "ros2_qos_adapter.hpp"
#include "resource_manager.hpp"

namespace performance_test
{

/// Communication plugin for Apex.OS using waitsets for the subscription side.
template<class Msg>
class ApexOSPollingSubscriptionSubscriber : public Subscriber
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename Msg::RosType;

  explicit ApexOSPollingSubscriptionSubscriber(const ExperimentConfiguration & ec)
  : m_node(ResourceManager::get().rclcpp_node()),
    m_ROS2QOSAdapter(ROS2QOSAdapter(ec.qos()).get()),
    m_polling_subscription(m_node->create_polling_subscription<DataType>(
        ec.topic_name() + ec.sub_topic_postfix(),
        m_ROS2QOSAdapter)),
    m_waitset(std::make_unique<rclcpp::Waitset<>>(m_polling_subscription))
  {
    if (ec.expected_num_pubs() > 0) {
      m_polling_subscription->wait_for_matched(
        ec.expected_num_pubs(),
        ec.expected_wait_for_matched_timeout(),
        std::greater_equal<size_t>(),
        0U,
        std::greater_equal<size_t>(),
        std::chrono::milliseconds(10 * ec.number_of_subscribers()));
    }
  }

  std::vector<ReceivedMsgStats> update_subscription() override
  {
    const auto wait_ret = m_waitset->wait(std::chrono::milliseconds(100), false);
    if (wait_ret.any()) {
      return take();
    } else {
      return {};
    }
  }

  std::vector<ReceivedMsgStats> take() override
  {
    m_subscriber_stats.clear();
    const auto loaned_msg = m_polling_subscription->take(RCLCPP_LENGTH_UNLIMITED);
    for (const auto msg : loaned_msg) {
      if (msg.info().valid()) {
        callback(msg.data());
      }
    }
    return m_subscriber_stats;
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  using PollingSubscriptionType = ::rclcpp::PollingSubscription<DataType>;
  std::shared_ptr<PollingSubscriptionType> m_polling_subscription;
  std::unique_ptr<rclcpp::Waitset<>> m_waitset;
  std::vector<ReceivedMsgStats> m_subscriber_stats;

  template<class T>
  void callback(const T & data)
  {
    const auto received_time = now_int64_t();
    static_assert(
      std::is_same<DataType,
      typename std::remove_cv<
        typename std::remove_reference<T>::type>::type>::value,
      "Parameter type passed to callback() does not match");
    m_subscriber_stats.emplace_back(
      data.time,
      received_time,
      data.id,
      sizeof(DataType)
    );
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__APEX_OS_POLLING_SUBSCRIPTION_COMMUNICATOR_HPP_
