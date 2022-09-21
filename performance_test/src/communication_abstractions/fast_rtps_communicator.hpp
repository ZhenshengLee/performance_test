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

#ifndef COMMUNICATION_ABSTRACTIONS__FAST_RTPS_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__FAST_RTPS_COMMUNICATOR_HPP_

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
#include <fastdds/dds/core/LoanableSequence.hpp>

#include <vector>

#include "communicator.hpp"
#include "resource_manager.hpp"
#include "../experiment_configuration/qos_abstraction.hpp"
#include "../utilities/msg_traits.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for FastRTPS.
class FastRTPSQOSAdapter
{
public:
  explicit FastRTPSQOSAdapter(const QOSAbstraction qos, bool interprocess)
  : m_qos(qos)
  , m_interprocess(interprocess)
  {}

  void apply(eprosima::fastdds::dds::DataWriterQos & wqos) const
  {
    apply_common(wqos);

    wqos.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
    wqos.reliable_writer_qos().times.heartbeatPeriod.fraction((200 * 1000 * 1000));
    wqos.publish_mode().kind = publish_mode();
  }

  void apply(eprosima::fastdds::dds::DataReaderQos & rqos) const
  {
    apply_common(rqos);
  }

private:
  const QOSAbstraction m_qos;
  const bool m_interprocess;

  template<typename EntityQos>
  void apply_common(EntityQos & eqos) const
  {
    eqos.history().kind = history_kind();
    eqos.history().depth = history_depth();
    eqos.resource_limits().max_samples = resource_limits_samples();
    eqos.resource_limits().allocated_samples = resource_limits_samples();
    eqos.reliability().kind = reliability();
    eqos.durability().kind = durability();
    eqos.data_sharing().automatic();
    if (!m_interprocess) eqos.data_sharing().off();
  }

  inline eprosima::fastrtps::ReliabilityQosPolicyKind reliability() const
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      return eprosima::fastrtps::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

  inline eprosima::fastrtps::DurabilityQosPolicyKind durability() const
  {
    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      return eprosima::fastrtps::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

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

  int32_t resource_limits_samples() const
  {
    return static_cast<int32_t>(m_qos.history_depth);
  }

  inline eprosima::fastrtps::PublishModeQosPolicyKind publish_mode() const
  {
    if (m_qos.sync_pubsub) {
      return eprosima::fastrtps::PublishModeQosPolicyKind::SYNCHRONOUS_PUBLISH_MODE;
    } else {
      return eprosima::fastrtps::PublishModeQosPolicyKind::ASYNCHRONOUS_PUBLISH_MODE;
    }
  }
};

template<class Topic>
class FastRTPSPublisher : public Publisher
{
public:
  using TopicType = typename Topic::EprosimaTopicType;
  using DataType = typename Topic::EprosimaType;

  explicit FastRTPSPublisher(const ExperimentConfiguration & ec)
  : m_resources(ResourceManager::get().fastdds_resources(
      eprosima::fastdds::dds::TypeSupport(new TopicType()))),
    m_datawriter(create_datawriter(m_resources, ec))
  {
  }

  void publish_copy(std::int64_t time, std::uint64_t sample_id) override
  {
    init_msg(m_data, time, sample_id);
    if (!m_datawriter->write(static_cast<void *>(&m_data))) {
      throw std::runtime_error("Failed to write sample");
    }
  }

  void publish_loaned(std::int64_t time, std::uint64_t sample_id) override
  {
    void * loaned_sample = nullptr;
    eprosima::fastrtps::types::ReturnCode_t ret = m_datawriter->loan_sample(loaned_sample);
    if (!ret) {
      throw std::runtime_error(
        "Failed to obtain a loaned sample, ERRORCODE is " + std::to_string(ret()));
    }
    DataType * sample = static_cast<DataType *>(loaned_sample);
    init_msg(*sample, time, sample_id);
    if (!m_datawriter->write(static_cast<void *>(sample))) {
      throw std::runtime_error("Failed to write sample");
    }
  }

private:
  ResourceManager::FastDDSGlobalResources m_resources;
  eprosima::fastdds::dds::DataWriter * m_datawriter;
  DataType m_data;

  static eprosima::fastdds::dds::DataWriter * create_datawriter(
    const ResourceManager::FastDDSGlobalResources & resources,
    const ExperimentConfiguration & ec
  )
  {
    const bool interprocess = ec.number_of_publishers() == 0 || ec.number_of_subscribers() == 0;
    const FastRTPSQOSAdapter qos(ec.qos(), interprocess);

    eprosima::fastdds::dds::DataWriterQos wqos;
    resources.publisher->get_default_datawriter_qos(wqos);
    qos.apply(wqos);

    auto writer = resources.publisher->create_datawriter(resources.topic, wqos);
    if (!writer) {
      throw std::runtime_error("failed to create datawriter");
    }

    return writer;
  }

  void init_msg(DataType & msg, std::int64_t time, std::uint64_t sample_id)
  {
    msg.time(time);
    msg.id(sample_id);
    MsgTraits::ensure_fixed_size(msg);
  }
};

template<class Topic>
class FastRTPSSubscriber : public Subscriber
{
public:
  using TopicType = typename Topic::EprosimaTopicType;
  using DataType = typename Topic::EprosimaType;

  explicit FastRTPSSubscriber(const ExperimentConfiguration & ec)
  : m_resources(ResourceManager::get().fastdds_resources(
      eprosima::fastdds::dds::TypeSupport(new TopicType()))),
    m_datareader(create_datareader(m_resources, ec))
  {
  }

  std::vector<ReceivedMsgStats> update_subscription() override
  {
    const eprosima::fastrtps::Duration_t secs_15{15, 0};
    m_datareader->wait_for_unread_message(secs_15);
    return take();
  }

  std::vector<ReceivedMsgStats> take() override
  {
    std::vector<ReceivedMsgStats> stats;

    FASTDDS_SEQUENCE(DataSeq, DataType);
    DataSeq data_seq;
    eprosima::fastdds::dds::SampleInfoSeq info_seq;

    const auto ok_ret_code = eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK;
    while (ok_ret_code == m_datareader->take(data_seq, info_seq, 1)) {
      const auto received_time = now_int64_t();
      if (info_seq[0].valid_data) {
        stats.emplace_back(
          data_seq[0].time(),
          received_time,
          data_seq[0].id(),
          sizeof(DataType)
        );
      }
      m_datareader->return_loan(data_seq, info_seq);
    }
    return stats;
  }

private:
  ResourceManager::FastDDSGlobalResources m_resources;
  eprosima::fastdds::dds::DataReader * m_datareader;

  static eprosima::fastdds::dds::DataReader * create_datareader(
    const ResourceManager::FastDDSGlobalResources & resources,
    const ExperimentConfiguration & ec
  )
  {
    const bool interprocess = ec.number_of_publishers() == 0 || ec.number_of_subscribers() == 0;
    const FastRTPSQOSAdapter qos(ec.qos(), interprocess);

    eprosima::fastdds::dds::DataReaderQos rqos;
    resources.subscriber->get_default_datareader_qos(rqos);
    qos.apply(rqos);

    auto reader = resources.subscriber->create_datareader(resources.topic, rqos);
    if (!reader) {
      throw std::runtime_error("failed to create datareader");
    }

    return reader;
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__FAST_RTPS_COMMUNICATOR_HPP_
