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

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/Domain.h>

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
  explicit FastRTPSQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}

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

private:
  const QOSAbstraction m_qos;
};

template<class Topic>
class FastRTPSPublisher : public Publisher
{
public:
  using TopicType = typename Topic::EprosimaTopicType;
  using DataType = typename Topic::EprosimaType;

  explicit FastRTPSPublisher(const ExperimentConfiguration & ec)
  : m_topic_type(new TopicType()),
    m_participant(ResourceManager::get().fastrtps_participant()),
    m_publisher(make_fastrtps_publisher(m_topic_type, m_participant, ec))
  {
  }

  void publish_copy(std::int64_t time, std::uint64_t sample_id) override
  {
    init_msg(m_data, time, sample_id);
    m_publisher->write(static_cast<void *>(&m_data));
  }

  void publish_loaned(std::int64_t, std::uint64_t) override
  {
    throw std::runtime_error("This plugin does not support zero copy transfer");
  }

private:
  TopicType * m_topic_type;
  eprosima::fastrtps::Participant * m_participant;
  eprosima::fastrtps::Publisher * m_publisher;
  DataType m_data;

  static eprosima::fastrtps::Publisher * make_fastrtps_publisher(
    TopicType * topic_type,
    eprosima::fastrtps::Participant * participant,
    const ExperimentConfiguration & ec
  )
  {
    eprosima::fastrtps::Domain::registerType(participant, topic_type);

    const FastRTPSQOSAdapter qos(ec.qos());

    eprosima::fastrtps::PublisherAttributes wparam;

    wparam.topic.topicKind = eprosima::fastrtps::rtps::TopicKind_t::NO_KEY;
    wparam.topic.topicDataType = topic_type->getName();
    wparam.topic.topicName = ec.topic_name() + ec.pub_topic_postfix();
    wparam.topic.historyQos.kind = qos.history_kind();
    wparam.topic.historyQos.depth = qos.history_depth();
    wparam.topic.resourceLimitsQos.max_samples = qos.resource_limits_samples();
    wparam.topic.resourceLimitsQos.allocated_samples = qos.resource_limits_samples();

    wparam.times.heartbeatPeriod.seconds = 2;
    wparam.times.heartbeatPeriod.fraction(200 * 1000 * 1000);

    wparam.qos.m_reliability.kind = qos.reliability();
    wparam.qos.m_durability.kind = qos.durability();
    wparam.qos.m_publishMode.kind = qos.publish_mode();

    return eprosima::fastrtps::Domain::createPublisher(participant, wparam);
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
  : m_topic_type(new TopicType()),
    m_participant(ResourceManager::get().fastrtps_participant()),
    m_subscriber(make_fastrtps_subscriber(m_topic_type, m_participant, ec))
  {
  }

  std::vector<ReceivedMsgStats> update_subscription() override
  {
    m_subscriber->waitForUnreadMessage();
    return take();
  }

  std::vector<ReceivedMsgStats> take() override
  {
    std::vector<ReceivedMsgStats> stats;
    while (m_subscriber->takeNextData(static_cast<void *>(&m_data), &m_info)) {
      const auto received_time = now_int64_t();
      if (m_info.sampleKind == eprosima::fastrtps::rtps::ChangeKind_t::ALIVE) {
        stats.emplace_back(
          m_data.time(),
          received_time,
          m_data.id(),
          sizeof(DataType)
        );
      }
    }
    return stats;
  }

private:
  TopicType * m_topic_type;
  eprosima::fastrtps::Participant * m_participant;
  eprosima::fastrtps::Subscriber * m_subscriber;

  DataType m_data;
  eprosima::fastrtps::SampleInfo_t m_info;

  static eprosima::fastrtps::Subscriber * make_fastrtps_subscriber(
    TopicType * topic_type,
    eprosima::fastrtps::Participant * participant,
    const ExperimentConfiguration & ec
  )
  {
    eprosima::fastrtps::Domain::registerType(participant, topic_type);

    const FastRTPSQOSAdapter qos(ec.qos());

    eprosima::fastrtps::SubscriberAttributes rparam;

    rparam.topic.topicKind = eprosima::fastrtps::rtps::TopicKind_t::NO_KEY;
    rparam.topic.topicDataType = topic_type->getName();
    rparam.topic.topicName = ec.topic_name() + ec.sub_topic_postfix();
    rparam.topic.historyQos.kind = qos.history_kind();
    rparam.topic.historyQos.depth = qos.history_depth();
    rparam.topic.resourceLimitsQos.max_samples = qos.resource_limits_samples();
    rparam.topic.resourceLimitsQos.allocated_samples = qos.resource_limits_samples();

    rparam.qos.m_reliability.kind = qos.reliability();
    rparam.qos.m_durability.kind = qos.durability();

    return eprosima::fastrtps::Domain::createSubscriber(participant, rparam);
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__FAST_RTPS_COMMUNICATOR_HPP_
