// Copyright 2022 Apex.AI, Inc.
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

#include <chrono>
#include <memory>
#include <ratio>
#include <utility>
#include <functional>

#ifndef COMMUNICATION_ABSTRACTIONS__APEX_OS_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__APEX_OS_COMMUNICATOR_HPP_

#include <apexcpp/node_state.hpp>
#include <cpputils/optional.hpp>
#include <executor/executable_item.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_qos_adapter.hpp"
#include "../experiment_metrics/publisher_stats.hpp"
#include "../experiment_metrics/subscriber_stats.hpp"

namespace performance_test
{
template <class MsgType>
class PublisherItem : public apex::executor::executable_item
{
public:
  PublisherItem(rclcpp::Node &node, apex::NodeState &node_state,
                PublisherStats &stats, const ExperimentConfiguration &ec)
      : apex::executor::executable_item(node, node_state),
        m_ec(ec),
        m_stats(stats),
        m_publisher(node.create_publisher<MsgType>(
            ec.topic_name() + ec.pub_topic_postfix(),
            ROS2QOSAdapter(ec.qos()).get()))
  {
    if (m_ec.expected_num_subs() > 0) {
      m_publisher->wait_for_matched(m_ec.expected_num_subs(),
                                    m_ec.expected_wait_for_matched_timeout());
    }
  }

private:
  void execute_impl() override {
    if (m_ec.is_zero_copy_transfer()) {
      if (!m_publisher->can_loan_messages()) {
        throw std::runtime_error(
            "RMW implementation does not support zero copy!");
      }
      auto borrowed_message{m_publisher->borrow_loaned_message()};
      init_msg(borrowed_message.get(), now_int64_t(), m_stats.next_sample_id());
      m_publisher->publish(std::move(borrowed_message));
      m_stats.update_publisher_stats();
    } else {
      init_msg(m_data, now_int64_t(), m_stats.next_sample_id());
      m_publisher->publish(m_data);
      m_stats.update_publisher_stats();
    }
  }

  inline
  void init_msg(MsgType & msg, std::int64_t time, std::uint64_t sample_id)
  {
    msg.time = time;
    msg.id = sample_id;
    init_bounded_sequence(msg);
    init_unbounded_sequence(msg);
    init_unbounded_string(msg);
  }

  template<typename T>
  inline
  std::enable_if_t<MsgTraits::has_bounded_sequence<T>::value, void>
  init_bounded_sequence(T & msg)
  {
    msg.bounded_sequence.resize(msg.bounded_sequence.capacity());
  }

  template<typename T>
  inline
  std::enable_if_t<!MsgTraits::has_bounded_sequence<T>::value, void>
  init_bounded_sequence(T &) {}

  template<typename T>
  inline
  std::enable_if_t<MsgTraits::has_unbounded_sequence<T>::value, void>
  init_unbounded_sequence(T & msg)
  {
    msg.unbounded_sequence.resize(m_ec.unbounded_msg_size());
  }

  template<typename T>
  inline
  std::enable_if_t<!MsgTraits::has_unbounded_sequence<T>::value, void>
  init_unbounded_sequence(T &) {}

  template<typename T>
  inline
  std::enable_if_t<MsgTraits::has_unbounded_string<T>::value, void>
  init_unbounded_string(T & msg)
  {
    msg.unbounded_string.resize(m_ec.unbounded_msg_size());
  }

  template<typename T>
  inline
  std::enable_if_t<!MsgTraits::has_unbounded_string<T>::value, void>
  init_unbounded_string(T &) {}

  MsgType m_data;
  const ExperimentConfiguration &m_ec;
  PublisherStats &m_stats;
  const typename rclcpp::Publisher<MsgType>::SharedPtr m_publisher;
};

template <class MsgType>
class SubscriberItem : public apex::executor::executable_item
{
public:
  SubscriberItem(rclcpp::Node &node, apex::NodeState &node_state,
                 SubscriberStats &stats, const ExperimentConfiguration &ec)
      : apex::executor::executable_item(node, node_state),
        m_ec(ec),
        m_stats(stats),
        m_subscription(node.template create_polling_subscription<MsgType>(
            ec.topic_name() + ec.sub_topic_postfix(),
            ROS2QOSAdapter(ec.qos()).get()))
  {
    if (m_ec.roundtrip_mode() ==
        ExperimentConfiguration::RoundTripMode::RELAY) {
      m_publisher.emplace(node.create_publisher<MsgType>(
          m_ec.topic_name() + m_ec.pub_topic_postfix(),
          ROS2QOSAdapter(m_ec.qos()).get()));
    }

    if (this->m_ec.expected_num_pubs() > 0) {
      m_subscription->wait_for_matched(
          this->m_ec.expected_num_pubs(),
          this->m_ec.expected_wait_for_matched_timeout(),
          std::greater_equal<size_t>(), 0U, std::greater_equal<size_t>(),
          std::chrono::milliseconds(10 * this->m_ec.number_of_subscribers()));
    }
  }

  apex::executor::subscription_list
  get_triggering_subscriptions_impl() const override
  {
    return {m_subscription};
  }

private:
  void execute_impl() override
  {
    const auto loaned_msg = m_subscription->take();
    const auto received_time = now_int64_t();
    for (const auto msg : loaned_msg) {
      if (msg.info().valid()) {
        handle_message(msg, received_time);
      }
    }
  }

  // Use data_copy() with unbounded message types
  template <typename T>
  auto handle_message(T &msg, const std::int64_t received_time)
      -> decltype(msg.data_copy(), void()) {
    this->template callback(msg.data_copy(), received_time);
  }

  // Use data() by default
  template <typename T>
  auto handle_message(T &msg, const std::int64_t received_time)
      -> decltype(msg.data(), void()) {
    this->template callback(msg.data(), received_time);
  }

  template <class T>
  void callback(const T &data, const std::int64_t received_time) {
    static_assert(
        std::is_same<MsgType,
                     typename std::remove_cv<
                         typename std::remove_reference<T>::type>::type>::value,
        "Parameter type passed to callback() does not match");
    if (m_ec.roundtrip_mode() ==
        ExperimentConfiguration::RoundTripMode::RELAY) {
      m_publisher.value()->publish(data);
    } else {
      m_stats.update_subscriber_stats(data.time, received_time, data.id,
                                      sizeof(MsgType));
    }
  }

  const ExperimentConfiguration &m_ec;
  SubscriberStats &m_stats;
  const typename rclcpp::PollingSubscription<MsgType>::SharedPtr m_subscription;
  apex::optional<typename rclcpp::Publisher<MsgType>::SharedPtr> m_publisher;
};

class ApexOsPublisherEntity {
public:
  virtual std::shared_ptr<apex::executor::executable_item>
  get_publisher_item() {
    return nullptr;
  }
};

class ApexOsSubscriberEntity {
public:
  virtual std::shared_ptr<apex::executor::executable_item>
  get_subscriber_item() {
    return nullptr;
  }
};

template <typename MsgType> class ApexOsPublisher : public ApexOsPublisherEntity
{
public:
  ApexOsPublisher(PublisherStats &stats, const ExperimentConfiguration &ec)
      : m_node("apex_os_publisher_node"),
        m_node_state(&m_node, std::chrono::seconds::max()),
        m_publisher_item(std::make_shared<PublisherItem<MsgType>>(
            m_node, m_node_state, stats, ec)) {}

  std::shared_ptr<apex::executor::executable_item>
  get_publisher_item() override
  {
    return m_publisher_item;
  }

private:
  rclcpp::Node m_node;
  apex::NodeState m_node_state;
  std::shared_ptr<PublisherItem<MsgType>> m_publisher_item;
};

template <typename MsgType> class ApexOsSubscriber : public ApexOsSubscriberEntity
{
public:
  ApexOsSubscriber(SubscriberStats &stats, const ExperimentConfiguration &ec)
      : m_node("apex_os_subscriber_node"),
        m_node_state(&m_node, std::chrono::seconds::max()),
        m_subscriber_item(std::make_shared<SubscriberItem<MsgType>>(
            m_node, m_node_state, stats, ec)) {}

  std::shared_ptr<apex::executor::executable_item>
  get_subscriber_item() override
  {
    return m_subscriber_item;
  }

private:
  rclcpp::Node m_node;
  apex::NodeState m_node_state;
  std::shared_ptr<SubscriberItem<MsgType>> m_subscriber_item;
};
}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__APEX_OS_COMMUNICATOR_HPP_
