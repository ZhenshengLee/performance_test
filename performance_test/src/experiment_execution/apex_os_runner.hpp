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

#ifndef EXPERIMENT_EXECUTION__APEX_OS_RUNNER_HPP_
#define EXPERIMENT_EXECUTION__APEX_OS_RUNNER_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>
#include <utility>

#include "runner.hpp"
#include "../execution_tasks/apex_os_entity_factory.hpp"
#include "../experiment_configuration/experiment_configuration.hpp"

#include <executor/executor_factory.hpp>
#include <executor/executor_runner.hpp>

namespace performance_test
{
class ApexOsRunner : public Runner
{
public:
  explicit ApexOsRunner(const ExperimentConfiguration & ec)
  : Runner(ec)
  {
    for (uint32_t i = 0; i < m_ec.number_of_publishers(); ++i) {
      m_pubs.push_back(performance_test::ApexOsEntityFactory::get_publisher(
          m_ec.msg_name(), m_pub_stats.at(i), m_ec));
    }

    for (uint32_t i = 0; i < m_ec.number_of_subscribers(); ++i) {
      m_subs.push_back(performance_test::ApexOsEntityFactory::get_subscriber(
          m_ec.msg_name(), m_sub_stats.at(i), m_ec));
    }
  }

protected:
  std::vector<std::shared_ptr<ApexOsPublisherEntity>> m_pubs;
  std::vector<std::shared_ptr<ApexOsSubscriberEntity>> m_subs;
};

class ApexOsSingleExecutorRunner : public ApexOsRunner {
public:
  explicit ApexOsSingleExecutorRunner(const ExperimentConfiguration & ec)
  : ApexOsRunner(ec),
    m_executor(apex::executor::executor_factory::create()),
    m_runner(apex::executor::executor_runner::deferred_tag(), *m_executor) {}

protected:
  virtual void run_pubs_and_subs()
  {
    for (auto &pub : m_pubs) {
      m_executor->add(std::move(pub->get_publisher_item()), m_ec.period_ns());
    }
    for (auto &sub : m_subs) {
      m_executor->add(std::move(sub->get_subscriber_item()));
    }
    m_runner.issue();
  }

private:
  const apex::executor::executor_ptr m_executor;
  const apex::executor::executor_runner m_runner;
};

class ApexOsExecutorPerCommunicatorRunner : public ApexOsRunner {
public:
  explicit ApexOsExecutorPerCommunicatorRunner(const ExperimentConfiguration & ec)
  : ApexOsRunner(ec) {}

protected:
  virtual void run_pubs_and_subs()
  {
    for (auto &sub : m_subs) {
      m_executors.emplace_back(apex::executor::executor_factory::create());
      m_executors.back()->add(std::move(sub->get_subscriber_item()), m_ec.period_ns());
      m_runners.emplace_back(*(m_executors.back()));
    }
    for (auto &pub : m_pubs) {
      m_executors.emplace_back(apex::executor::executor_factory::create());
      m_executors.back()->add(std::move(pub->get_publisher_item()), m_ec.period_ns());
      m_runners.emplace_back(*(m_executors.back()));
    }
  }

private:
  std::vector<apex::executor::executor_ptr> m_executors;
  std::vector<apex::executor::executor_runner> m_runners;
};

class ApexOsSingleExecutorChainRunner : public ApexOsRunner {
public:
  explicit ApexOsSingleExecutorChainRunner(const ExperimentConfiguration & ec)
  : ApexOsRunner(ec),
    m_executor(apex::executor::executor_factory::create()),
    m_runner(apex::executor::executor_runner::deferred_tag(), *m_executor)
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

protected:
  virtual void run_pubs_and_subs()
  {
    std::vector<apex::executor::executable_item_ptr> chain_of_nodes;
    chain_of_nodes.reserve(m_pubs.size() + m_subs.size());
    for (auto & pub : m_pubs) {
      chain_of_nodes.push_back(pub->get_publisher_item());
    }
    for (auto & sub : m_subs) {
      chain_of_nodes.push_back(sub->get_subscriber_item());
    }
    m_executor->add(chain_of_nodes, m_ec.period_ns());
    m_runner.issue();
  }

private:
  const apex::executor::executor_ptr m_executor;
  const apex::executor::executor_runner m_runner;
};

}  // namespace performance_test

#endif  // EXPERIMENT_EXECUTION__APEX_OS_RUNNER_HPP_
