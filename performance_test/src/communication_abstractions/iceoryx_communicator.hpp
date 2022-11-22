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

#ifndef COMMUNICATION_ABSTRACTIONS__ICEORYX_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__ICEORYX_COMMUNICATOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include <iceoryx_posh/popo/untyped_publisher.hpp>
#include <iceoryx_posh/popo/untyped_subscriber.hpp>

#include "communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{
class IceoryxCommunicator
{
public:
  IceoryxCommunicator() {
    ResourceManager::get().init_iceoryx_runtime();
  }
};

template<class Msg>
class IceoryxPublisher : public IceoryxCommunicator, public Publisher
{
public:
  using DataType = typename Msg::RosType;

  explicit IceoryxPublisher(const ExperimentConfiguration & ec)
  : m_publisher(
      iox::capro::ServiceDescription{
        iox::capro::IdString_t{iox::cxx::TruncateToCapacity, Msg::msg_name()},
        iox::capro::IdString_t{iox::cxx::TruncateToCapacity, ec.topic_name()},
        iox::capro::IdString_t{"Object"}
      },
      iox::popo::PublisherOptions{}) {}

  void publish_copy(std::int64_t time, std::uint64_t sample_id) override
  {
    publish_loaned(time, sample_id);
  }

  void publish_loaned(std::int64_t time, std::uint64_t sample_id) override
  {
    m_publisher.loan(sizeof(DataType))
    .and_then(
      [&, this](auto & userPayload) {
        auto sample = static_cast<DataType *>(userPayload);
        this->init_msg(*sample, time, sample_id);
        m_publisher.publish(userPayload);
      })
    .or_else(
      [](auto &) {
        throw std::runtime_error("Failed to write to sample");
      });
  }

private:
  iox::popo::UntypedPublisher m_publisher;
  DataType m_data;
};

template<class Msg>
class IceoryxSubscriber : public IceoryxCommunicator, public Subscriber
{
public:
  using DataType = typename Msg::RosType;

  explicit IceoryxSubscriber(const ExperimentConfiguration & ec)
  : m_subscriber(
      iox::capro::ServiceDescription{
        iox::capro::IdString_t{iox::cxx::TruncateToCapacity, Msg::msg_name()},
        iox::capro::IdString_t{iox::cxx::TruncateToCapacity, ec.topic_name()},
        iox::capro::IdString_t{"Object"}
      },
      subscriber_options(ec))
  {
    m_waitset.attachEvent(m_subscriber, iox::popo::SubscriberEvent::DATA_RECEIVED)
    .or_else(
      [](
        auto) {
        std::cerr << "unable to attach Event DATA_RECEIVED to iceoryx Waitset" << std::endl;
        std::exit(EXIT_FAILURE);
      });
  }

  std::vector<ReceivedMsgStats> update_subscription() override
  {
    std::vector<ReceivedMsgStats> stats;
    if (m_subscriber.getSubscriptionState() == iox::SubscribeState::SUBSCRIBED) {
      auto eventVector = m_waitset.timedWait(iox::units::Duration::fromSeconds(15));
      for (auto & event : eventVector) {
        if (event->doesOriginateFrom(&m_subscriber)) {
          while (m_subscriber.hasData()) {
            for (const auto & s : take()) {
              stats.push_back(s);
            }
          }
        }
      }
    } else {
      std::cout << "Not subscribed!" << std::endl;
    }
    return stats;
  }

  std::vector<ReceivedMsgStats> take() override
  {
    std::vector<ReceivedMsgStats> stats;
    m_subscriber.take()
    .and_then(
      [this, &stats](const void * data) {
        const auto received_time = now_int64_t();
        auto receivedSample = static_cast<const DataType *>(data);
        stats.emplace_back(
          receivedSample->time,
          received_time,
          receivedSample->id,
          sizeof(DataType)
        );
        m_subscriber.release(data);
      })
    .or_else(
      [](auto & result) {
        std::cerr << result << std::endl;
        if (result !=
        iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE)
        {
          throw std::runtime_error(
            "Error: received Chunk not available");
        }
      });
    return stats;
  }

private:
  static iox::popo::SubscriberOptions subscriber_options(const ExperimentConfiguration & ec) {
    iox::popo::SubscriberOptions options;
    options.queueCapacity = ec.qos().history_depth;
    return options;
  }

  iox::popo::UntypedSubscriber m_subscriber;
  iox::popo::WaitSet<> m_waitset;
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__ICEORYX_COMMUNICATOR_HPP_
