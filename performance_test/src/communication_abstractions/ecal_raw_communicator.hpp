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

#ifndef COMMUNICATION_ABSTRACTIONS__ECAL_RAW_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__ECAL_RAW_COMMUNICATOR_HPP_

#include <ecal/ecal.h>

#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "communicator.hpp"
#include "resource_manager.hpp"
#include "../experiment_configuration/qos_abstraction.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for eCAL.
class EcalRawQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit EcalRawQOSAdapter(const QOSAbstraction qos)
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
class EcalRawCommunicator : public Communicator
{
public:
  /// The data type to use.
  using DataType = typename Msg::RosType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit EcalRawCommunicator(SpinLock & lock)
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
    if (!m_publisher) {
      const EcalRawQOSAdapter qos(m_ec.qos());
      ResourceManager::get().init_ecal_runtime();
      eCAL::QOS::SWriterQOS writerQos;
      writerQos.history_kind       = qos.history_kind();
      writerQos.reliability        = qos.reliability();
      writerQos.history_kind_depth = qos.history_depth();
      m_publisher = std::unique_ptr<eCAL::CPublisher>(new eCAL::CPublisher(m_ec.topic_name()));
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

    // write time and id into message
    lock();
    init_msg(m_data, time);
    increment_sent();  // We increment before publishing so we don't have to lock twice.
    unlock();
    m_publisher->Send(&m_data, sizeof(DataType));
  }

  // subscriber callback function
  void on_receive(const struct eCAL::SReceiveCallbackData* data_)
  {
    // read time and id out of the received buffer
    std::unique_lock<std::mutex> lk(m_cb_mtx);
    lock();
    DataType* data_type_ptr = static_cast<DataType*>(data_->buf);
    m_data.time = data_type_ptr->time;
    m_data.id   = data_type_ptr->id;
    unlock();
    // signal update_subscription to process
    m_cb_cv.notify_one();
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
    if (!m_subscriber) {
      const EcalRawQOSAdapter qos(m_ec.qos());
      ResourceManager::get().init_ecal_runtime();
      eCAL::QOS::SReaderQOS readerQos;
      readerQos.history_kind       = qos.history_kind();
      readerQos.reliability        = qos.reliability();
      readerQos.history_kind_depth = qos.history_depth();
      m_subscriber = std::unique_ptr<eCAL::CSubscriber>(new eCAL::CSubscriber(m_ec.topic_name()));
      // https://github.com/ZhenshengLee/performance_test/issues/1
      // m_subscriber->SetQOS(readerQos);
      // add receive callback
      m_subscriber->AddReceiveCallback(std::bind(&EcalRawCommunicator::on_receive, this, std::placeholders::_2));
    }

    // did we get new receives ?
    std::unique_lock<std::mutex> lk(m_cb_mtx);
    if(m_cb_cv.wait_for(lk, std::chrono::milliseconds(m_timeout_ms)) ==  std::cv_status::no_timeout) {
      lock();
      if (m_prev_timestamp >= m_data.time) {
        throw std::runtime_error(
                "Data consistency violated. Received sample with not strictly "
                "older timestamp. Time diff: " + std::to_string(
                  m_data.time - m_prev_timestamp) + " Data Time: " +
                std::to_string(m_data.time)
        );
      }
      if (m_ec.roundtrip_mode() == ExperimentConfiguration::RoundTripMode::RELAY) {
        unlock();
        publish(m_data.time);
        lock();
      } else {
        m_prev_timestamp = m_data.time;
        update_lost_samples_counter(m_data.id);
        add_latency_to_statistics(m_data.time);
        increment_received();
      }
      unlock();
    }
  }

  /// Returns the data received in bytes.
  std::size_t data_received()
  {
    return num_received_samples() * sizeof(DataType);
  }

private:
  std::unique_ptr<eCAL::CPublisher>  m_publisher;
  std::unique_ptr<eCAL::CSubscriber> m_subscriber;
  std::mutex                         m_cb_mtx;
  std::condition_variable            m_cb_cv;
  int                                m_timeout_ms = 0;
  DataType                           m_data;
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__ECAL_RAW_COMMUNICATOR_HPP_
