// Copyright 2022 zhenshenglee, Inc.
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

#ifndef COMMUNICATION_ABSTRACTIONS__ECAL_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__ECAL_COMMUNICATOR_HPP_

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>

#include <string>
#include <memory>

#include "communicator.hpp"
#include "resource_manager.hpp"
#include "../experiment_configuration/qos_abstraction.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for eCAL.
class EcalQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit EcalQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}

  /// Returns derived eCAL reliability setting from the stored abstract QOS setting.
  inline eCAL::QOS::eQOSPolicy_Reliability reliability() const
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      return eCAL::QOS::eQOSPolicy_Reliability::best_effort_reliability_qos;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      return eCAL::QOS::eQOSPolicy_Reliability::reliable_reliability_qos;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived eCAL history policy setting from the stored abstract QOS setting.
  inline eCAL::QOS::eQOSPolicy_HistoryKind history_kind() const
  {
    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      return eCAL::QOS::eQOSPolicy_HistoryKind::keep_all_history_qos;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      return eCAL::QOS::eQOSPolicy_HistoryKind::keep_last_history_qos;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived eCAL history depth setting from the stored abstract QOS setting.
  int32_t history_depth() const
  {
    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      return static_cast<int32_t>(m_qos.history_depth);
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      // Keep all, keeps all. No depth required, but setting to dummy value.
      return 1;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

private:
  const QOSAbstraction m_qos;
};


/**
 * \brief The plugin for eCAL.
 * \tparam Msg The msg type to use.
 */
template<class Msg>
class EcalCommunicator : public Communicator
{
public:
  /// The data type to use.
  using DataType = typename Msg::EcalType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit EcalCommunicator(SpinLock & lock)
  : Communicator(lock),
    m_publisher(nullptr),
    m_subscriber(nullptr)
  {
    auto hz = static_cast<double>(this->m_ec.rate());
    auto period_ms = 1000.0 / hz;
    this->m_timeout_ms = std::max(10 * period_ms, 100.0);
  }

  /**
   * \brief Publishes the provided data.
   *
   *  The first time this function is called it also creates the publisher.
   *  Further it updates all internal counters while running.
   * \param data The data to publish.
   * \param time The time to fill into the data field.
   */
  void publish(std::int64_t time)
  {
    if (m_publisher == nullptr) {
      const EcalQOSAdapter qos(m_ec.qos());
      ResourceManager::get().init_ecal_runtime();
      eCAL::QOS::SWriterQOS writerQos;
      writerQos.history_kind = qos.history_kind();
      writerQos.reliability = qos.reliability();
      writerQos.history_kind_depth = qos.history_depth();
      m_publisher = std::unique_ptr<eCAL::protobuf::CPublisher<DataType>>(
        new eCAL::protobuf::CPublisher<DataType>(m_ec.topic_name()));
      // https://github.com/ZhenshengLee/performance_test/issues/1
      m_publisher->SetQOS(writerQos);
    }
    // set number of publisher memory buffers to improve performance
    // https://github.com/ZhenshengLee/performance_test/issues/1
    m_publisher->ShmSetBufferCount(1);

    if (m_ec.is_zero_copy_transfer()) {
      // enable zero copy mode
      m_publisher->ShmEnableZeroCopy(1);
    }

    lock();
    init_msg(m_data, time);
    increment_sent();  // We increment before publishing so we don't have to lock twice.
    unlock();
    m_publisher->Send(m_data);
  }

  /**
   * \brief Reads received data.
   *
   * The first time this function is called it also creates the subscriber.
   * In detail this function:
   * * Reads samples.
   * * Verifies that the data arrived in the right order, chronologically and also
   *   consistent with the publishing order.
   * * Counts received and lost samples.
   * * Calculates the latency of the samples received and updates the statistics
       accordingly.
   */
  void update_subscription()
  {
    if (m_subscriber == nullptr) {
      const EcalQOSAdapter qos(m_ec.qos());
      ResourceManager::get().init_ecal_runtime();
      eCAL::QOS::SReaderQOS ReaderQos;
      ReaderQos.history_kind = qos.history_kind();
      ReaderQos.reliability = qos.reliability();
      ReaderQos.history_kind_depth = qos.history_depth();
      m_subscriber = std::unique_ptr<eCAL::protobuf::CSubscriber<DataType>>(
        new eCAL::protobuf::CSubscriber<DataType>(m_ec.topic_name()));
      // https://github.com/ZhenshengLee/performance_test/issues/1
      m_subscriber->SetQOS(ReaderQos);
    }

    bool success = m_subscriber->Receive(m_data, nullptr, m_timeout_ms);
    if(success)
    {
      lock();
      if (m_prev_timestamp >= m_data.time()) {
        throw std::runtime_error(
                "Data consistency violated. Received sample with not strictly "
                "older timestamp. Time diff: " + std::to_string(
                  m_data.time() - m_prev_timestamp) + " Data Time: " +
                std::to_string(m_data.time())
        );
      }
      if (m_ec.roundtrip_mode() == ExperimentConfiguration::RoundTripMode::RELAY) {
        unlock();
        publish(m_data.time());
        lock();
      } else {
        m_prev_timestamp = m_data.time();
        update_lost_samples_counter(m_data.id());
        add_latency_to_statistics(m_data.time());
        increment_received();
      }
    }
    unlock();
  }

  /// Returns the data received in bytes.
  std::size_t data_received()
  {
    return num_received_samples() * sizeof(DataType);
  }

private:
  std::unique_ptr<eCAL::protobuf::CPublisher<DataType>> m_publisher;
  std::unique_ptr<eCAL::protobuf::CSubscriber<DataType>> m_subscriber;

  DataType m_data;

  int m_timeout_ms;

  void init_msg(DataType & msg, std::int64_t time)
  {
    msg.clear_msg_array_size();
    msg.set_msg_array_size(msg.msg_array_size());
    // fill with 0
    msg.mutable_array()->Resize(msg.msg_array_size(), 0);
    msg.set_time(time);
    msg.set_id(next_sample_id());
    ensure_fixed_size(msg);
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__ECAL_COMMUNICATOR_HPP_