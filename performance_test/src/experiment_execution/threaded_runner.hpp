// Copyright 2017-2022 Apex.AI, Inc.
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

#ifndef EXPERIMENT_EXECUTION__THREADED_RUNNER_HPP_
#define EXPERIMENT_EXECUTION__THREADED_RUNNER_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

#include "runner.hpp"
#include "../communication_abstractions/communicator_factory.hpp"
#include "../execution_tasks/publisher_task.hpp"
#include "../execution_tasks/subscriber_task.hpp"
#include "../execution_tasks/round_trip_relay_task.hpp"
#include "../experiment_configuration/experiment_configuration.hpp"

namespace performance_test
{
class DataEntityRunner : public Runner
{
public:
  explicit DataEntityRunner(const ExperimentConfiguration & ec)
  : Runner(ec)
  {
    for (uint32_t i = 0; i < m_ec.number_of_publishers(); ++i) {
      m_pubs.push_back(
        std::make_shared<PublisherTask>(
          ec,
          m_pub_stats.at(i),
          CommunicatorFactory::get_publisher(ec)));
    }
    for (uint32_t i = 0; i < m_ec.number_of_subscribers(); ++i) {
      m_subs.push_back(
        std::make_shared<SubscriberTask>(
          ec,
          m_sub_stats.at(i),
          CommunicatorFactory::get_subscriber(ec)));
    }
  }

  virtual ~DataEntityRunner() {}

protected:
  std::vector<std::shared_ptr<PublisherTask>> m_pubs;
  std::vector<std::shared_ptr<SubscriberTask>> m_subs;
};

class InterThreadRunner : public DataEntityRunner
{
public:
  explicit InterThreadRunner(const ExperimentConfiguration & ec)
  : DataEntityRunner(ec)
  {}

  ~InterThreadRunner()
  {
    for (auto & thread : m_thread_pool) {
      thread.join();
    }
  }

protected:
  virtual void run_pubs_and_subs()
  {
    m_thread_pool.reserve(m_pubs.size() + m_subs.size());

    for (auto & sub : m_subs) {
      m_thread_pool.emplace_back(
        [&sub, this]() {
          while (m_running) {
            sub->run();
          }
        });
    }

    for (auto & pub : m_pubs) {
      m_thread_pool.emplace_back(
        [&pub, this]() {
          while (m_running) {
            pub->run();
          }
        });
    }
  }

private:
  std::vector<std::thread> m_thread_pool;
};

class IntraThreadRunner : public DataEntityRunner
{
public:
  explicit IntraThreadRunner(const ExperimentConfiguration & ec)
  : DataEntityRunner(ec)
  {
    if (ec.number_of_publishers() != 1) {
      throw std::invalid_argument(
              "Intra-thread execution requires exactly one publisher.");
    }
    if (ec.number_of_subscribers() < 1) {
      throw std::invalid_argument(
              "Intra-thread execution requires at least one subscriber.");
    }
    if (!ec.is_zero_copy_transfer()) {
      throw std::invalid_argument(
              "Intra-thread execution only works with loaned messages (zero copy).");
    }
    if (ec.roundtrip_mode() != ExperimentConfiguration::RoundTripMode::NONE) {
      throw std::invalid_argument(
              "Intra-thread execution only works with RoundTripMode NONE.");
    }
  }

  ~IntraThreadRunner()
  {
    m_thread->join();
  }

protected:
  virtual void run_pubs_and_subs()
  {
    m_thread = std::make_unique<std::thread>(
      [this]() {
        while (m_running) {
          for (auto & pub : m_pubs) {
            pub->run();
          }
          for (auto & sub : m_subs) {
            sub->take();
          }
        }
      }
    );
  }

private:
  std::unique_ptr<std::thread> m_thread;
};

class RoundTripMainRunner : public InterThreadRunner
{
public:
  explicit RoundTripMainRunner(const ExperimentConfiguration & ec)
  : InterThreadRunner(ec)
  {
    if (ec.number_of_publishers() != 1) {
      throw std::invalid_argument(
              "Round-trip main requires exactly one publisher.");
    }
    if (ec.number_of_subscribers() != 1) {
      throw std::invalid_argument(
              "Round-trip main requires exactly one subscriber.");
    }
    if (ec.is_zero_copy_transfer()) {
      throw std::invalid_argument(
              "Round-trip main can not use loaned messages (zero copy).");
    }
  }
};

class RoundTripRelayRunner : public Runner
{
public:
  explicit RoundTripRelayRunner(const ExperimentConfiguration & ec)
  : Runner(ec),
    m_relay(std::make_shared<RoundTripRelayTask>(
        ec,
        CommunicatorFactory::get_publisher(ec),
        CommunicatorFactory::get_subscriber(ec)))
  {
    if (ec.number_of_publishers() != 1) {
      throw std::invalid_argument(
              "Round-trip relay requires exactly one publisher.");
    }
    if (ec.number_of_subscribers() != 1) {
      throw std::invalid_argument(
              "Round-trip relay requires exactly one subscriber.");
    }
    if (ec.is_zero_copy_transfer()) {
      throw std::invalid_argument(
              "Round-trip relay can not use loaned messages (zero copy).");
    }
  }

  virtual ~RoundTripRelayRunner()
  {
    m_thread->join();
  }

protected:
  virtual void run_pubs_and_subs()
  {
    m_thread = std::make_unique<std::thread>(
      [this]() {
        while (m_running) {
          m_relay->run();
        }
      }
    );
  }

private:
  std::shared_ptr<RoundTripRelayTask> m_relay;
  std::unique_ptr<std::thread> m_thread;
};

}  // namespace performance_test

#endif  // EXPERIMENT_EXECUTION__THREADED_RUNNER_HPP_
