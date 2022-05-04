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

#ifndef COMMUNICATION_ABSTRACTIONS__RCLCPP_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__RCLCPP_COMMUNICATOR_HPP_


#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <atomic>
#include <utility>

#include "../experiment_configuration/qos_abstraction.hpp"

#include "communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for ROS 2.
class ROS2QOSAdapter
{
public:
  /**
   * \brief The constructor which will save the provided abstract QOS.
   * \param qos The abstract QOS to derive the settings from.
   */
  explicit ROS2QOSAdapter(const QOSAbstraction qos)
  : m_qos(qos) {}
  /// Gets a ROS 2 QOS profile derived from the stored abstract QOS.
  inline rclcpp::QoS get() const
  {
    rclcpp::QoS ros_qos(m_qos.history_depth);

    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      ros_qos.best_effort();
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      ros_qos.reliable();
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      ros_qos.durability_volatile();
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      ros_qos.transient_local();
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      ros_qos.keep_all();
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      ros_qos.keep_last(m_qos.history_depth);
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    return ros_qos;
  }

private:
  const QOSAbstraction m_qos;
};

/// Communication plugin interface for ROS 2 for the subscription side.
template<class Msg>
class RclcppCommunicator : public Communicator
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename Msg::RosType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit RclcppCommunicator(SpinLock & lock)
  : Communicator(lock),
    m_node(ResourceManager::get().rclcpp_node()),
    m_ROS2QOSAdapter(ROS2QOSAdapter(m_ec.qos()).get()) {}

  /**
   * \brief Publishes the provided data.
   *
   *  The first time this function is called it also creates the publisher.
   *  Further it updates all internal counters while running.
   *
   * \param data The data to publish.
   * \param time The time to fill into the data field.
   */
  void publish(std::int64_t time)
  {
    if (!m_publisher) {
      auto ros2QOSAdapter = m_ROS2QOSAdapter;
      m_publisher = m_node->create_publisher<DataType>(
        m_ec.topic_name() + m_ec.pub_topic_postfix(), ros2QOSAdapter);
    }
    if (m_ec.is_zero_copy_transfer()) {
      #ifdef PERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED
      if (!m_publisher->can_loan_messages()) {
        throw std::runtime_error("RMW implementation does not support zero copy!");
      }
      auto borrowed_message{m_publisher->borrow_loaned_message()};
      lock();
      init_msg(borrowed_message.get(), time);
      increment_sent();  // We increment before publishing so we don't have to lock twice.
      unlock();
      m_publisher->publish(std::move(borrowed_message));
      #else
      throw std::runtime_error("ROS2 distribution does not support zero copy!");
      #endif
    } else {
      lock();
      init_msg(m_data, time);
      increment_sent();  // We increment before publishing so we don't have to lock twice.
      unlock();
      m_publisher->publish(m_data);
    }
  }

  /// Reads received data from ROS 2 using callbacks
  virtual void update_subscription() = 0;

  /// Returns the accumulated data size in bytes.
  std::size_t data_received()
  {
    return num_received_samples() * sizeof(DataType);
  }

protected:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  /**
   * \brief Callback handler which handles the received data.
   *
   * * Verifies that the data arrived in the right order, chronologically and also consistent with the publishing order.
   * * Counts recieved and lost samples.
   * * Calculates the latency of the samples received and updates the statistics accordingly.
   *
   * \param data The data received.
   */
  void callback(const typename DataType::SharedPtr data)
  {
    callback(*data);
  }

  template<class T>
  void callback(const T & data)
  {
    static_assert(
      std::is_same<DataType,
      typename std::remove_cv<typename std::remove_reference<T>::type>::type>::value,
      "Parameter type passed to callback() does not match");
    if (m_prev_timestamp >= data.time) {
      throw std::runtime_error(
              "Data consistency violated. Received sample with not strictly older timestamp");
    }

    if (m_ec.roundtrip_mode() == ExperimentConfiguration::RoundTripMode::RELAY) {
      publish(data.time);
    } else {
      lock();
      m_prev_timestamp = data.time;
      update_lost_samples_counter(data.id);
      add_latency_to_statistics(data.time);
      increment_received();
      unlock();
    }
  }

private:
  std::shared_ptr<::rclcpp::Publisher<DataType>> m_publisher;

  DataType m_data;

  inline
  void init_msg(DataType & msg, std::int64_t time)
  {
    msg.time = time;
    msg.id = next_sample_id();
    init_bounded_sequence(msg);
    init_unbounded_sequence(msg);
    init_unbounded_string(msg);
  }

  template<typename T>
  inline
  std::enable_if_t<has_bounded_sequence<T>::value, void>
  init_bounded_sequence(T & msg)
  {
    msg.bounded_sequence.resize(msg.bounded_sequence.capacity());
  }

  template<typename T>
  inline
  std::enable_if_t<!has_bounded_sequence<T>::value, void>
  init_bounded_sequence(T &) {}

  template<typename T>
  inline
  std::enable_if_t<has_unbounded_sequence<T>::value, void>
  init_unbounded_sequence(T & msg)
  {
    msg.unbounded_sequence.resize(m_ec.unbounded_msg_size());
  }

  template<typename T>
  inline
  std::enable_if_t<!has_unbounded_sequence<T>::value, void>
  init_unbounded_sequence(T &) {}

  template<typename T>
  inline
  std::enable_if_t<has_unbounded_string<T>::value, void>
  init_unbounded_string(T & msg)
  {
    msg.unbounded_string.resize(m_ec.unbounded_msg_size());
  }

  template<typename T>
  inline
  std::enable_if_t<!has_unbounded_string<T>::value, void>
  init_unbounded_string(T &) {}
};
}  // namespace performance_test
#endif  // COMMUNICATION_ABSTRACTIONS__RCLCPP_COMMUNICATOR_HPP_
