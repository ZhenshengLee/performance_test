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

#ifndef COMMUNICATION_ABSTRACTIONS__FAST_DDS_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__FAST_DDS_COMMUNICATOR_HPP_

// common/src/eProsima/Fast-DDS/examples/C++/DDS
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/core/LoanableConstSequence.hpp>

#include <atomic>

#include "communicator.hpp"
#include "resource_manager.hpp"
#include "../experiment_configuration/qos_abstraction.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for FastDDS.
class FastDDSQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit FastDDSQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}

  /// Returns derived FastDDS reliability setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::ReliabilityQosPolicy reliability() const
  {
    eprosima::fastrtps::ReliabilityQosPolicy reliability;
    reliability.kind = reliability_kind();
    return reliability;
  }
  /// Returns derived FastDDS reliability setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::ReliabilityQosPolicyKind reliability_kind() const
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived FastDDS durability setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::DurabilityQosPolicy durability() const
  {
    eprosima::fastrtps::DurabilityQosPolicy durability;
    durability.kind = durability_kind();
    return durability;
  }
  /// Returns derived FastDDS durability setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::DurabilityQosPolicyKind durability_kind() const
  {
    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived FastDDS history policy setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::HistoryQosPolicy history() const
  {
    eprosima::fastrtps::HistoryQosPolicy history;
    history.kind = history_kind();
    history.depth = history_depth();
    return history;
  }
  /// Returns derived FastDDS history kind policy setting from the stored abstract QOS setting.
  inline eprosima::fastrtps::HistoryQosPolicyKind history_kind() const
  {
    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      return eprosima::fastrtps::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      return eprosima::fastrtps::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }
  /// Returns derived FastDDS history depth setting from the stored abstract QOS setting.
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
  /// Returns the number of samples to be allocated on the history
  inline eprosima::fastrtps::ResourceLimitsQosPolicy resource_limits() const
  {
    eprosima::fastrtps::ResourceLimitsQosPolicy resourcelimits;
    resourcelimits.max_samples = resource_limits_samples();
    resourcelimits.allocated_samples = resource_limits_samples();
    return resourcelimits;
  }
  /// Returns the number of samples to be allocated on the history
  int32_t resource_limits_samples() const
  {
    return static_cast<int32_t>(m_qos.history_depth);
  }
  /// Returns the publish mode policy from the stored abstract QOS setting.
  inline eprosima::fastrtps::PublishModeQosPolicy publish_mode() const
  {
    eprosima::fastrtps::PublishModeQosPolicy publish_mode;
    publish_mode.kind = publish_mode_kind();
    return publish_mode;
  }
  /// Returns the publish mode policy from the stored abstract QOS setting.
  inline eprosima::fastrtps::PublishModeQosPolicyKind publish_mode_kind() const
  {
    if (m_qos.sync_pubsub) {
      return eprosima::fastrtps::PublishModeQosPolicyKind::SYNCHRONOUS_PUBLISH_MODE;
    } else {
      return eprosima::fastrtps::PublishModeQosPolicyKind::ASYNCHRONOUS_PUBLISH_MODE;
    }
  }

private:
  const QOSAbstraction m_qos;
};

/**
 * \brief Communication plugin for FastDDS.
 * \tparam Topic The topic type to use.
 *
 * The code in there is derived from
 * https://github.com/eProsima/Fast-DDS/tree/master/examples/C%2B%2B/HelloWorldExample.
 */
template<class Topic>
class FastDDSCommunicator : public Communicator
{
public:
  /// The topic type to use.
  using TopicType = typename Topic::FastddsTopicType;
  /// The data type to publish and subscribe to.
  using DataType = typename Topic::FastddsType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit FastDDSCommunicator(SpinLock & lock)
  : Communicator(lock),
    m_publisher(nullptr),
    m_subscriber(nullptr),
    m_writer(nullptr),
    m_reader(nullptr),
    m_topic(nullptr),
    m_topic_type(new TopicType())
  {
    m_participant = ResourceManager::get().fastdds_participant();
    if (!s_type_registered) {
      s_type_registered = true;
      m_topic_type.register_type(m_participant);
    }
    auto hz = static_cast<double>(this->m_ec.rate());
    auto period = std::chrono::duration<double>(1.0 / hz);
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
    auto timeout_ns = std::max(10 * period_ns, std::chrono::nanoseconds(100 * 1000 * 1000));
    this->m_timeout = eprosima::fastrtps::Duration_t(0, timeout_ns.count());
  }

  /**
   * \brief Publishes the provided data.
   *
   *  The first time this function is called it also creates the data writer.
   *  Further it updates all internal counters while running.
   * \param data The data to publish.
   * \param time The time to fill into the data field.
   */
  void publish(std::int64_t time)
  {
    if (!m_writer) {
      const FastDDSQOSAdapter qos(m_ec.qos());
      // create publisher
      // include/fastdds/dds/publisher/qos/PublisherQos.hpp
      m_publisher = m_participant->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
      if(!m_publisher) {
        throw std::runtime_error("Failed to create publisher! ");
      }
      // create topic
      // common/src/eProsima/Fast-DDS/include/fastdds/dds/topic/qos/TopicQos.hpp
      eprosima::fastdds::dds::TopicQos tqos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
      // qst: difference between topicqos and writerqos?
      // tqos.history(qos.history());
      // tqos.resource_limits(qos.resource_limits());
      // tqos.reliability(qos.reliability());
      // tqos.durability(qos.durability());
      if(!m_topic) {
        m_topic = m_participant->create_topic(m_ec.topic_name() + m_ec.pub_topic_postfix(), m_topic_type->getName(), tqos);
      }
      // create datawriter
      eprosima::fastdds::dds::DataWriterQos wqos = m_publisher->get_default_datawriter_qos();
      if(m_ec.is_zero_copy_transfer()){
        wqos.history().depth = qos.history_depth();
        wqos.durability().kind = eprosima::fastrtps::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
        wqos.data_sharing().on("shared_directory");
      } else {
        wqos.history().kind = qos.history_kind();
        wqos.history().depth = qos.history_depth();
        wqos.resource_limits().max_samples = qos.resource_limits_samples();
        wqos.resource_limits().allocated_samples = qos.resource_limits_samples();
        wqos.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
        wqos.reliable_writer_qos().times.heartbeatPeriod.fraction((200 * 1000 * 1000));
        wqos.reliability().kind = qos.reliability_kind();
        wqos.publish_mode().kind = qos.publish_mode_kind();
        wqos.data_sharing().automatic();
      }
      m_writer = m_publisher->create_datawriter(m_topic, wqos);
      // remain
      // wparam.topic.topicKind = eprosima::fastrtps::rtps::TopicKind_t::NO_KEY;
    }
    if (m_ec.is_zero_copy_transfer()) {
      void* loaned_sample = nullptr;
      eprosima::fastrtps::types::ReturnCode_t ret = m_writer->loan_sample(loaned_sample);
      if (!ret) {
        throw std::runtime_error("Failed to obtain a loaned sample, ERRORCODE is " + std::to_string(ret()));
      }
      DataType* sample = static_cast<DataType*>(loaned_sample);
      lock();
      init_msg(*sample, time);
      increment_sent();  // We increment before publishing so we don't have to lock twice.
      unlock();
      m_writer->write(sample);
    } else {
      lock();
      init_msg(m_data, time);
      increment_sent();  // We increment before publishing so we don't have to lock twice.
      unlock();
      m_writer->write(static_cast<void *>(&m_data));
    }

  }
  /**
   * \brief Reads received data from DDS.
   *
   * In detail this function:
   * * Reads samples from DDS.
   * * Verifies that the data arrived in the right order, chronologically and also consistent with the publishing order.
   * * Counts received and lost samples.
   * * Calculates the latency of the samples received and updates the statistics accordingly.
   */
  void update_subscription()
  {
    if (!m_reader) {
      const FastDDSQOSAdapter qos(m_ec.qos());
      // create subscriber
      m_subscriber = m_participant->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
      if(!m_subscriber) {
        throw std::runtime_error("Failed to create subscriber! ");
      }
      // create topic
      // common/src/eProsima/Fast-DDS/include/fastdds/dds/topic/qos/TopicQos.hpp
      eprosima::fastdds::dds::TopicQos tqos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
      // qst: difference between topicqos and writerqos?
      // tqos.history(qos.history());
      // tqos.resource_limits(qos.resource_limits());
      // tqos.reliability(qos.reliability());
      // tqos.durability(qos.durability());
      if(!m_topic) {
        m_topic = m_participant->create_topic(m_ec.topic_name() + m_ec.pub_topic_postfix(), m_topic_type->getName(), tqos);
      }
      // create datareader
      eprosima::fastdds::dds::DataReaderQos rqos = m_subscriber->get_default_datareader_qos();;
      if(m_ec.is_zero_copy_transfer()){
        rqos.history().depth = qos.history_depth();
        rqos.reliability().kind = eprosima::fastrtps::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
        rqos.durability().kind = eprosima::fastrtps::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
        rqos.data_sharing().on("shared_directory");
      } else {
        rqos.history().kind = qos.history_kind();
        rqos.history().depth = qos.history_depth();
        rqos.resource_limits().max_samples = qos.resource_limits_samples();
        rqos.resource_limits().allocated_samples = qos.resource_limits_samples();
        rqos.reliability().kind = qos.reliability_kind();
        rqos.data_sharing().automatic();
      }
      m_reader = m_subscriber->create_datareader(m_topic, rqos);

      // remain
      // rparam.topic.topicKind = eprosima::fastrtps::rtps::TopicKind_t::NO_KEY;
    }

    m_reader->wait_for_unread_message(m_timeout);
    lock();
    if (m_ec.is_zero_copy_transfer()) {
      FASTDDS_CONST_SEQUENCE(DataSeq, DataType);
      DataSeq data;
      while (eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK == m_reader->take_next_instance(data, m_infos)) {
        for (eprosima::fastdds::dds::LoanableCollection::size_type i = 0; i < m_infos.length(); ++i)
        {
          if (m_infos[i].valid_data)
          {
            m_data = data[i];
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
        }
        m_reader->return_loan(data, m_infos);
      }
    } else {
      while (eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK == m_reader->take_next_sample(static_cast<void *>(&m_data), &m_info)) {
        if (m_info.instance_state == eprosima::fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE) {
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
  eprosima::fastdds::dds::DomainParticipant * m_participant;
  eprosima::fastdds::dds::Publisher * m_publisher;
  eprosima::fastdds::dds::DataWriter *m_writer;
  eprosima::fastdds::dds::Subscriber * m_subscriber;
  eprosima::fastdds::dds::DataReader *m_reader;
  eprosima::fastdds::dds::Topic *m_topic;
  eprosima::fastdds::dds::SampleInfo m_info;
  eprosima::fastdds::dds::SampleInfoSeq m_infos;

  static bool s_type_registered;

  eprosima::fastdds::dds::TypeSupport m_topic_type;
  DataType m_data;

  void init_msg(DataType & msg, std::int64_t time)
  {
    msg.time(time);
    msg.id(next_sample_id());
    ensure_fixed_size(msg);
  }

  eprosima::fastrtps::Duration_t m_timeout;
};

template<class Topic>
bool FastDDSCommunicator<Topic>::s_type_registered = false;

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__FAST_DDS_COMMUNICATOR_HPP_
