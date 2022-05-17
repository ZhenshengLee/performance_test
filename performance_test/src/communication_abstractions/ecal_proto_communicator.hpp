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

#ifndef COMMUNICATION_ABSTRACTIONS__ECAL_PROTO_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__ECAL_PROTO_COMMUNICATOR_HPP_

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
class EcalProtoQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit EcalProtoQOSAdapter(const QOSAbstraction qos)
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
class EcalProtoCommunicator : public Communicator
{
public:
  /// The data type to use.
  using DataType = typename Msg::EcalType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit EcalProtoCommunicator(SpinLock & lock)
  : Communicator(lock),
    m_publisher(nullptr),
    m_subscriber(nullptr)
  {
    auto hz = static_cast<double>(this->m_ec.rate());
    auto period_ms = 1000.0 / hz;
    this->m_timeout_ms = static_cast<int>(std::max(10 * period_ms, 100.0));
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
      const EcalProtoQOSAdapter qos(m_ec.qos());
      ResourceManager::get().init_ecal_runtime();
      eCAL::QOS::SWriterQOS writerQos;
      writerQos.history_kind       = qos.history_kind();
      writerQos.reliability        = qos.reliability();
      writerQos.history_kind_depth = qos.history_depth();
      m_publisher = std::unique_ptr<eCAL::protobuf::CPublisher<DataType>>(
        new eCAL::protobuf::CPublisher<DataType>(m_ec.topic_name()));
      // https://github.com/ZhenshengLee/performance_test/issues/1
      // m_publisher->SetQOS(writerQos);
      // set number of used publisher memory buffer
      // https://github.com/ZhenshengLee/performance_test/issues/1
      m_publisher->ShmSetBufferCount(1);
      if (m_ec.is_zero_copy_transfer()){
        // enable zero copy mode
        m_publisher->ShmEnableZeroCopy(1);
      } else {
        // disable zero copy mode
        m_publisher->ShmEnableZeroCopy(0);
      }
    }

    lock();
    init_msg(m_data, time);
    increment_sent();  // We increment before publishing so we don't have to lock twice.
    unlock();
    m_publisher->Send(m_data);
  }

  // subscriber callback function
  void on_receive(const DataType& data_)
  {
    lock();
    // read time and id out of the received message
    m_data.set_time(data_.time());
    m_data.set_id(data_.id());
    unlock();
    // signal update_subscription to process
    gSetEvent(m_event);
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
      const EcalProtoQOSAdapter qos(m_ec.qos());
      ResourceManager::get().init_ecal_runtime();
      eCAL::QOS::SReaderQOS readerQos;
      readerQos.history_kind       = qos.history_kind();
      readerQos.reliability        = qos.reliability();
      readerQos.history_kind_depth = qos.history_depth();
      m_subscriber = std::unique_ptr<eCAL::protobuf::CSubscriber<DataType>>(
        new eCAL::protobuf::CSubscriber<DataType>(m_ec.topic_name()));
      // https://github.com/ZhenshengLee/performance_test/issues/1
      // m_subscriber->SetQOS(readerQos);
      // add receive callback
      m_subscriber->AddReceiveCallback(std::bind(&EcalProtoCommunicator::on_receive, this, std::placeholders::_2));
      // create sync event
      eCAL::gOpenEvent(&m_event);
    }

    if( gWaitForEvent(m_event, m_timeout_ms) ) {
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
      unlock();
    }
  }

  /// Returns the data received in bytes.
  std::size_t data_received()
  {
    // still not correct but better then sizeof(DataType)
    return num_received_samples() * m_data.msg_array_size() * sizeof(google::protobuf::uint32);
  }

private:
  std::unique_ptr<eCAL::protobuf::CPublisher<DataType>>  m_publisher;
  std::unique_ptr<eCAL::protobuf::CSubscriber<DataType>> m_subscriber;

  eCAL::EventHandleT                                     m_event;
  int                                                    m_timeout_ms;
  DataType                                               m_data;

  void init_msg(DataType & msg, std::int64_t time)
  {
    // set array size and fill it with 0
    if(msg.array().size() < msg.msg_array_size_e_MAX) {
      msg.set_msg_array_size(msg.msg_array_size_e_MAX);
      msg.mutable_array()->Resize(msg.msg_array_size_e_MAX, 0);
    }
    // set time and id
    msg.set_time(time);
    msg.set_id(next_sample_id());
    ensure_fixed_size(msg);
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__ECAL_PROTO_COMMUNICATOR_HPP_
