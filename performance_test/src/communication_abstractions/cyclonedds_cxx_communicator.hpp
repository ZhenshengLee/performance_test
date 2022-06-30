// Copyright 2021 Apex.AI, Inc.
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

#ifndef COMMUNICATION_ABSTRACTIONS__CYCLONEDDS_CXX_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__CYCLONEDDS_CXX_COMMUNICATOR_HPP_

#include <memory>
#include <string>

#include <dds/dds.hpp>

#include "communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{

/**
 * \brief Apply the QOSAbstraction to the data reader or writer QOS.
 *
 * \param rw_qos The data reader or writer QOS.
 * \param ec The experiment configuration.
 */
template<class CycloneDDSCXXQos>
void apply_cylonedds_cxx_qos(
  CycloneDDSCXXQos & rw_qos,
  const ExperimentConfiguration & ec
)
{
  const QOSAbstraction qos = ec.qos();

  if (qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
    rw_qos.policy(dds::core::policy::Reliability::BestEffort());
  } else if (qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
    rw_qos.policy(dds::core::policy::Reliability::Reliable());
  } else {
    throw std::runtime_error("Unsupported QOS!");
  }

  if (qos.durability == QOSAbstraction::Durability::VOLATILE) {
    rw_qos.policy(dds::core::policy::Durability::Volatile());
  } else if (qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
    rw_qos.policy(dds::core::policy::Durability::TransientLocal());
  } else {
    throw std::runtime_error("Unsupported QOS!");
  }

  if (qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
    rw_qos.policy(dds::core::policy::History::KeepAll());
  } else if (qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
    rw_qos.policy(dds::core::policy::History::KeepLast(qos.history_depth));
  } else {
    throw std::runtime_error("Unsupported QOS!");
  }
}

/**
 * \brief Creates a CycloneDDS-CXX data writer.
 */
template<typename DataType>
dds::pub::DataWriter<DataType> make_cyclonedds_cxx_datawriter(
  const dds::domain::DomainParticipant & participant,
  const dds::pub::Publisher & publisher,
  const ExperimentConfiguration & ec
)
{
  std::string topic_name = ec.topic_name() + ec.pub_topic_postfix();
  auto topic = dds::topic::Topic<DataType>(participant, topic_name);

  dds::pub::qos::DataWriterQos dw_qos = publisher.default_datawriter_qos();
  apply_cylonedds_cxx_qos(dw_qos, ec);

  return dds::pub::DataWriter<DataType>(publisher, topic, dw_qos);
}

/**
 * \brief Creates a CycloneDDS-CXX data reader.
 */
template<typename DataType>
dds::sub::DataReader<DataType> make_cyclonedds_cxx_datareader(
  const dds::domain::DomainParticipant & participant,
  const dds::sub::Subscriber & subscriber,
  const ExperimentConfiguration & ec
)
{
  std::string topic_name = ec.topic_name() + ec.sub_topic_postfix();
  auto topic = dds::topic::Topic<DataType>(participant, topic_name);

  dds::sub::qos::DataReaderQos dr_qos = subscriber.default_datareader_qos();
  apply_cylonedds_cxx_qos(dr_qos, ec);

  return dds::sub::DataReader<DataType>(subscriber, topic, dr_qos);
}


/**
 * \brief The plugin for CycloneDDS-CXX.
 * \tparam Msg The msg type to use.
 */
template<class Msg>
class CycloneDDSCXXCommunicator : public Communicator
{
public:
  /// The data type to use.
  using DataType = typename Msg::CycloneDDSCXXType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit CycloneDDSCXXCommunicator(DataStats & stats)
  : Communicator(stats),
    m_participant(ResourceManager::get().cyclonedds_cxx_participant()),
    m_publisher(m_participant), m_subscriber(m_participant),
    m_datawriter(make_cyclonedds_cxx_datawriter<DataType>(
        m_participant, m_publisher, m_ec)),
    m_datareader(make_cyclonedds_cxx_datareader<DataType>(
        m_participant, m_subscriber, m_ec)),
    m_read_condition(m_datareader,
      dds::sub::status::SampleState::not_read()),
    m_waitset()
  {
    m_waitset.attach_condition(m_read_condition);

    if (m_ec.is_zero_copy_transfer() && !m_datawriter.delegate()->is_loan_supported()) {
      throw std::runtime_error("Zero-copy transfer is not supported.");
    }
  }

  ~CycloneDDSCXXCommunicator()
  {
    this->m_datareader = dds::core::null;
    this->m_datawriter = dds::core::null;
    this->m_subscriber = dds::core::null;
    this->m_publisher = dds::core::null;
  }

  void publish(std::int64_t time) override
  {
    if (m_ec.is_zero_copy_transfer()) {
      DataType & loaned_sample = m_datawriter.delegate()->loan_sample();
      m_stats.lock();
      init_msg(loaned_sample, time);
      m_stats.update_publisher_stats();
      m_stats.unlock();
      m_datawriter->write(loaned_sample);
    } else {
      DataType sample;
      m_stats.lock();
      init_msg(sample, time);
      m_stats.update_publisher_stats();
      m_stats.unlock();
      m_datawriter->write(sample);
    }
  }

  void update_subscription() override
  {
    // Wait for the data to become available. This is the only condition, so no need to inspect the
    // returned list of triggered conditions.
    try {
      m_waitset.wait(dds::core::Duration(15, 0));
    } catch (dds::core::TimeoutError &) {
      // The timeout probably comes from reaching the maximum runtime
      return;
    }
    dds::sub::LoanedSamples<DataType> samples = m_datareader->take();
    const auto received_time = m_stats.now();
    for (auto & sample : samples) {
      if (sample->info().valid()) {
        if (m_ec.roundtrip_mode() == ExperimentConfiguration::RoundTripMode::RELAY) {
          publish(sample->data().time());
        } else {
          m_stats.lock();
          m_stats.update_subscriber_stats(
            sample->data().time(), received_time, sample->data().id(), sizeof(DataType));
          m_stats.unlock();
        }
      }
    }
  }

private:
  dds::domain::DomainParticipant m_participant;
  dds::pub::Publisher m_publisher;
  dds::sub::Subscriber m_subscriber;
  dds::pub::DataWriter<DataType> m_datawriter;
  dds::sub::DataReader<DataType> m_datareader;
  dds::sub::cond::ReadCondition m_read_condition;
  dds::core::cond::WaitSet m_waitset;

  void init_msg(DataType & msg, std::int64_t time)
  {
    msg.time(time);
    msg.id(m_stats.next_sample_id());
    MsgTraits::ensure_fixed_size(msg);
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__CYCLONEDDS_CXX_COMMUNICATOR_HPP_
