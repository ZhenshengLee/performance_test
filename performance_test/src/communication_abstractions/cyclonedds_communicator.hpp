// Copyright 2019 ADLINK Techonology, Inc.
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

#ifndef COMMUNICATION_ABSTRACTIONS__CYCLONEDDS_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__CYCLONEDDS_COMMUNICATOR_HPP_

#include <dds/dds.h>
#include <dds/ddsc/dds_loan_api.h>

#include <string>
#include <vector>

#include "communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{

/**
 * \brief Translates abstract QOS settings to specific QOS settings for Cyclone DDS
 * data writers and readers.
 *
 * The reason that this class is constructed like this is that one usually gets a
 * partially specified QOS from the topic or similar higher level entity and just
 * changes some settings from these.
 */
class CycloneDDSQOSAdapter
{
public:
  explicit CycloneDDSQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}

  template<class CycloneDDSQos>
  void apply(CycloneDDSQos & qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      dds_qset_history(qos, DDS_HISTORY_KEEP_ALL, 1);
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, static_cast<int32_t>(m_qos.history_depth));
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }
  }

private:
  const QOSAbstraction m_qos;
};

/**
 * \brief Translates abstract QOS settings to specific QOS settings for iceoryx
 * data writers and readers, for Cyclone DDS zero copy
 *
 * The reason that this class is constructed like this is that one usually gets a
 * partially specified QOS from the topic or similar higher level entity and just
 * changes some settings from these.
 */
class CycloneDDSIceoryxQOSAdapter
{
public:
  explicit CycloneDDSIceoryxQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}

  template<class CycloneDDSQos>
  void apply(CycloneDDSQos & qos)
  {
    std::cerr << "Cyclone DDS + iceoryx only supports RELIABLE reliability. " <<
      "Setting reliability to RELIABLE." << std::endl;
    dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));

    std::cerr << "Cyclone DDS + iceoryx only supports VOLATILE durability. " <<
      "Setting durability to VOLATILE." << std::endl;
    dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);

    std::cerr << "Cyclone DDS + iceoryx only supports KEEP_LAST history. " <<
      "Setting history to KEEP_LAST, with a depth of 16." << std::endl;
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, static_cast<int32_t>(16));
  }

private:
  const QOSAbstraction m_qos;
};

template<class Msg>
class CycloneDDSPublisher : public Publisher
{
public:
  using DataType = typename Msg::CycloneDDSType;

  explicit CycloneDDSPublisher(const ExperimentConfiguration & ec)
  : m_participant(ResourceManager::get().cyclonedds_participant()),
    m_datawriter(create_datawriter(ec, m_participant)) {}

  void publish_copy(std::int64_t time, std::uint64_t sample_id) override
  {
    init_msg(m_data, time, sample_id);
    if (dds_write(m_datawriter, static_cast<void *>(&m_data)) < 0) {
      throw std::runtime_error("Failed to write to sample");
    }
  }

  void publish_loaned(std::int64_t time, std::uint64_t sample_id) override
  {
    void * loaned_sample;
    dds_return_t status = dds_loan_sample(m_datawriter, &loaned_sample);
    if (status != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to obtain a loaned sample " + std::to_string(status));
    }
    DataType * sample = static_cast<DataType *>(loaned_sample);
    init_msg(*sample, time, sample_id);
    status = dds_write(m_datawriter, sample);
    if (status == DDS_RETCODE_UNSUPPORTED) {
      throw std::runtime_error("DDS write unsupported");
    } else if (status != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to write to sample");
    }
  }

private:
  dds_entity_t m_participant;
  dds_entity_t m_datawriter;
  DataType m_data;

  static dds_entity_t create_datawriter(
    const ExperimentConfiguration & ec,
    dds_entity_t participant
  )
  {
    dds_qos_t * dw_qos = dds_create_qos();
    if (ec.is_zero_copy_transfer()) {
      CycloneDDSIceoryxQOSAdapter qos_adapter(ec.qos());
      qos_adapter.apply(dw_qos);
    } else {
      CycloneDDSQOSAdapter qos_adapter(ec.qos());
      qos_adapter.apply(dw_qos);
    }

    dds_entity_t topic = dds_create_topic(
      participant,
      Msg::CycloneDDSDesc(),
      (ec.topic_name() + ec.pub_topic_postfix()).c_str(),
      nullptr,
      nullptr);

    dds_entity_t datawriter = dds_create_writer(participant, topic, dw_qos, nullptr);

    dds_delete(topic);
    dds_delete_qos(dw_qos);

    if (datawriter < 0) {
      throw std::runtime_error("failed to create datawriter");
    }
    return datawriter;
  }
};

template<class Msg>
class CycloneDDSSubscriber : public Subscriber
{
public:
  using DataType = typename Msg::CycloneDDSType;

  explicit CycloneDDSSubscriber(const ExperimentConfiguration & ec)
  : m_participant(ResourceManager::get().cyclonedds_participant()),
    m_datareader(create_datareader(ec, m_participant)),
    m_waitset(create_waitset(m_participant, m_datareader)) {}

  std::vector<ReceivedMsgStats> update_subscription() override
  {
    dds_waitset_wait(m_waitset, nullptr, 0, DDS_SECS(15));
    return take();
  }

  std::vector<ReceivedMsgStats> take() override
  {
    std::vector<ReceivedMsgStats> stats;
    void * untyped = nullptr;
    dds_sample_info_t si;
    int32_t n;
    while ((n = dds_take(m_datareader, &untyped, &si, 1, 1)) > 0) {
      const auto received_time = now_int64_t();
      const DataType * data = static_cast<DataType *>(untyped);
      if (si.valid_data) {
        stats.emplace_back(
          data->time,
          received_time,
          data->id,
          sizeof(DataType)
        );
      }
      dds_return_loan(m_datareader, &untyped, n);
    }
    return stats;
  }

private:
  dds_entity_t m_participant;
  dds_entity_t m_datareader;
  dds_entity_t m_waitset;
  dds_entity_t m_condition;

  static dds_entity_t create_datareader(
    const ExperimentConfiguration & ec,
    dds_entity_t participant
  )
  {
    dds_qos_t * dw_qos = dds_create_qos();
    if (ec.is_zero_copy_transfer()) {
      CycloneDDSIceoryxQOSAdapter qos_adapter(ec.qos());
      qos_adapter.apply(dw_qos);
    } else {
      CycloneDDSQOSAdapter qos_adapter(ec.qos());
      qos_adapter.apply(dw_qos);
    }

    dds_entity_t topic = dds_create_topic(
      participant,
      Msg::CycloneDDSDesc(),
      (ec.topic_name() + ec.sub_topic_postfix()).c_str(),
      nullptr,
      nullptr);

    dds_entity_t datareader = dds_create_reader(participant, topic, dw_qos, nullptr);

    dds_delete(topic);
    dds_delete_qos(dw_qos);

    if (datareader < 0) {
      throw std::runtime_error("failed to create datareader");
    }
    return datareader;
  }

  static dds_entity_t create_waitset(
    dds_entity_t participant,
    dds_entity_t datareader)
  {
    dds_set_status_mask(datareader, DDS_DATA_AVAILABLE_STATUS);
    dds_entity_t waitset = dds_create_waitset(participant);
    if (dds_waitset_attach(waitset, datareader, 1) < 0) {
      throw std::runtime_error("failed to attach waitset");
    }
    return waitset;
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__CYCLONEDDS_COMMUNICATOR_HPP_
