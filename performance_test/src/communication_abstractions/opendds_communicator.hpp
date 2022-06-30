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

#ifndef COMMUNICATION_ABSTRACTIONS__OPENDDS_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__OPENDDS_COMMUNICATOR_HPP_

#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/WaitSet.h>

#include "communicator.hpp"
#include "resource_manager.hpp"

#ifdef LENGTH_UNLIMITED
#undef LENGTH_UNLIMITED
#endif

#define BLOCK_SEC 10
#define BLOCK_NANO_SEC 0

namespace performance_test
{

/**
 * \brief Translates abstract QOS settings to specific QOS settings for OpenDDS data writers and readers.
 *
 * The reason that this class is constructed like this is that one usually gets a partially specified QOS from the topic
 * or similar higher level entity and just changes some settings from these.
 */

class OpenDdsQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit OpenDdsQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}
  /**
   * \brief  Applies the abstract QOS to an existing QOS leaving unsupported values as they were.
   * \tparam ConnextDDSMicroQos The type of the QOS setting, for example data reader or data writer QOS.
   * \param qos The QOS settings to fill supported values in.
   */

  void apply_dr(DDS::DataReaderQos & qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      qos.history.kind = DDS::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.depth = static_cast<decltype(qos.history.depth)>(m_qos.history_depth);
    } else {
      // Keep all, keeps all. No depth required.
    }
  }

  void apply_dw(DDS::DataWriterQos & qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS::VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS::TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      qos.history.kind = DDS::KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS::KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.depth = static_cast<decltype(qos.history.depth)>(m_qos.history_depth);
    } else {
      // Keep all, keeps all. No depth required.
    }
  }

private:
  const QOSAbstraction m_qos;
};

/**
 * \brief The plugin for OpenDDS.
 * \tparam Topic The topic type to use.
 */
template<class Topic>
class OpenDDSCommunicator : public Communicator
{
public:
  /// The data type to use.
  using DataType = typename Topic::OpenDDSTopicType;
  /// The type of the data writer.
  using DataWriterType = typename Topic::OpenDDSDataWriterType;
  /// The type of the data reader.
  using DataReaderType = typename Topic::OpenDDSDataReaderType;
  /// The type of a sequence of data.
  using DataTypeSeq = typename Topic::OpenDDSDataTypeSeq;

  explicit OpenDDSCommunicator(DataStats & stats)
  : Communicator(stats), m_datawriter(nullptr), m_datareader(nullptr),
    m_typed_datareader(nullptr)
  {
    m_participant = ResourceManager::get().opendds_participant();
    register_topic();
  }

  void publish(std::int64_t time) override
  {
    if (m_datawriter == nullptr) {
      DDS::Publisher_ptr publisher;
      DDS::DataWriterQos dw_qos;
      ResourceManager::get().opendds_publisher(publisher, dw_qos);

      OpenDdsQOSAdapter qos_adapter(m_ec.qos());
      qos_adapter.apply_dw(dw_qos);

      m_datawriter = publisher->create_datawriter(
        m_topic,
        dw_qos, nullptr, OpenDDS::DCPS::DEFAULT_STATUS_MASK);
      if (CORBA::is_nil(m_datawriter)) {
        throw std::runtime_error("Could not create datawriter");
      }

      m_typed_datawriter = DataWriterType::_narrow(m_datawriter);
      if (CORBA::is_nil(m_typed_datawriter)) {
        throw std::runtime_error("failed datawriter narrow");
      }
    }
    if (m_ec.is_zero_copy_transfer()) {
      throw std::runtime_error("This plugin does not support zero copy transfer");
    }
    m_stats.lock();
    init_msg(m_data, time);
    m_stats.update_publisher_stats();
    m_stats.unlock();
    auto retcode = m_typed_datawriter->write(m_data, DDS::HANDLE_NIL);
    if (retcode != DDS::RETCODE_OK) {
      throw std::runtime_error("Failed to write to sample");
    }
  }

  void update_subscription() override
  {
    if (CORBA::is_nil(m_datareader)) {
      DDS::Subscriber_ptr subscriber = nullptr;
      DDS::DataReaderQos dr_qos;
      ResourceManager::get().opendds_subscriber(subscriber, dr_qos);

      OpenDdsQOSAdapter qos_adapter(m_ec.qos());
      qos_adapter.apply_dr(dr_qos);

      /* Only DDS_DATA_AVAILABLE_STATUS supported currently */
      m_datareader = subscriber->create_datareader(
        m_topic,
        dr_qos,
        nullptr,
        OpenDDS::DCPS::DEFAULT_STATUS_MASK);

      if (CORBA::is_nil(m_datareader)) {
        throw std::runtime_error("datareader == nullptr");
      }

      m_condition = m_datareader->get_statuscondition();
      m_condition->set_enabled_statuses(DDS::DATA_AVAILABLE_STATUS);
      m_waitset.attach_condition(m_condition);

      m_typed_datareader = DataReaderType::_narrow(m_datareader);
      if (m_typed_datareader == nullptr) {
        throw std::runtime_error("m_typed_datareader == nullptr");
      }
    }

    DDS::Duration_t wait_timeout = {15, 0};
    m_waitset.wait(m_condition_seq, wait_timeout);

    auto ret = m_typed_datareader->take(
      m_data_seq, m_sample_info_seq, DDS::LENGTH_UNLIMITED,
      DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE,
      DDS::ANY_INSTANCE_STATE);
    const auto received_time = m_stats.now();
    if (ret == DDS::RETCODE_OK) {
      m_stats.lock();
      for (decltype(m_data_seq.length()) j = 0; j < m_data_seq.length(); ++j) {
        const auto & data = m_data_seq[j];
        if (m_sample_info_seq[j].valid_data) {
          m_stats.check_data_consistency(data.time);
          m_stats.update_subscriber_stats(data.time, received_time, data.id, sizeof(DataType));
        }
      }
      m_stats.unlock();

      if (m_ec.roundtrip_mode() == ExperimentConfiguration::RoundTripMode::RELAY) {
        throw std::runtime_error("Round trip mode is not implemented for OpenDDS!");
      }

      m_typed_datareader->return_loan(
        m_data_seq,
        m_sample_info_seq);
    }
  }

private:
  /// Registers a topic to the participant. It makes sure that each topic is only registered once.
  void register_topic()
  {
    if (CORBA::is_nil(m_topic)) {
      DDS::ReturnCode_t retcode;
      retcode = Topic::get_type_support()->register_type(
        m_participant,
        Topic::msg_name().c_str());
      if (retcode != DDS::RETCODE_OK) {
        throw std::runtime_error("failed to register type");
      }
      m_topic = m_participant->create_topic(
        m_ec.topic_name().c_str(),
        Topic::msg_name().c_str(),
        TOPIC_QOS_DEFAULT,
        nullptr,
        OpenDDS::DCPS::DEFAULT_STATUS_MASK);
      if (CORBA::is_nil(m_topic)) {
        throw std::runtime_error("topic == nullptr");
      }
    }
  }

  DDS::DomainParticipant_ptr m_participant;

  DDS::DataWriter_ptr m_datawriter;
  DDS::DataReader_ptr m_datareader;

  DDS::WaitSet m_waitset;
  DDS::StatusCondition_ptr m_condition;
  DDS::ConditionSeq m_condition_seq;

  DataReaderType * m_typed_datareader;
  DataWriterType * m_typed_datawriter;

  DataTypeSeq m_data_seq;
  DDS::SampleInfoSeq m_sample_info_seq;
  static DDS::Topic_ptr m_topic;

  DataType m_data;
};

template<class Topic>
DDS::Topic_ptr OpenDDSCommunicator<Topic>::m_topic = nullptr;

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__OPENDDS_COMMUNICATOR_HPP_
