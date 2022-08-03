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

#ifndef EXECUTION_TASKS__SUBSCRIBER_TASK_HPP_
#define EXECUTION_TASKS__SUBSCRIBER_TASK_HPP_

#include <functional>
#include <memory>
#include <thread>

#include "../experiment_metrics/subscriber_stats.hpp"
#include "../utilities/memory_checker.hpp"
#include "../utilities/spin_lock.hpp"

namespace performance_test
{
class SubscriberTask
{
public:
  SubscriberTask(
    const ExperimentConfiguration & ec,
    SubscriberStats & stats,
    std::shared_ptr<Subscriber> sub)
  : m_stats(stats),
    m_sub(sub),
    m_memory_checker(ec) {}

  SubscriberTask & operator=(const SubscriberTask &) = delete;
  SubscriberTask(const SubscriberTask &) = delete;

  void run()
  {
    for (const auto & stats : m_sub->update_subscription()) {
      m_stats.update_subscriber_stats(stats);
    }
    m_memory_checker.enable_memory_tools_checker();
  }

  void take()
  {
    for (const auto & stats : m_sub->take()) {
      m_stats.update_subscriber_stats(stats);
    }
    m_memory_checker.enable_memory_tools_checker();
  }

private:
  SubscriberStats & m_stats;
  std::shared_ptr<Subscriber> m_sub;
  MemoryChecker m_memory_checker;
};

}  // namespace performance_test

#endif  // EXECUTION_TASKS__SUBSCRIBER_TASK_HPP_
