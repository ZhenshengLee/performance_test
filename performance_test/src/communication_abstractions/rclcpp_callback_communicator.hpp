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

#ifndef COMMUNICATION_ABSTRACTIONS__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__RCLCPP_CALLBACK_COMMUNICATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <atomic>

#include "../experiment_configuration/topics.hpp"

#include "rclcpp_communicator.hpp"
#include "resource_manager.hpp"

namespace performance_test
{
/// Communication plugin for ROS 2 using ROS 2 callbacks for the subscription side.
template<class Msg, class Executor>
class RclcppCallbackCommunicator : public RclcppCommunicator<Msg>
{
public:
  /// The data type to publish and subscribe to.
  using DataType = typename RclcppCommunicator<Msg>::DataType;

  /// Constructor which takes a reference \param lock to the lock to use.
  explicit RclcppCallbackCommunicator(SpinLock & lock)
  : RclcppCommunicator<Msg>(lock),
    m_subscription(nullptr)
  {
    m_executor.add_node(this->m_node);
  }

  /// Reads received data from ROS 2 using callbacks
  void update_subscription() override
  {
    if (!m_subscription) {
      m_subscription = this->m_node->template create_subscription<DataType>(
        this->m_ec.topic_name() + this->m_ec.sub_topic_postfix(), this->m_ROS2QOSAdapter,
        [this](const typename DataType::SharedPtr data) {this->callback(data);});
    }
    this->lock();
    m_executor.spin_once(std::chrono::milliseconds(100));
    this->unlock();
  }

private:
  Executor m_executor;
  std::shared_ptr<::rclcpp::Subscription<DataType>> m_subscription;
};

template<class Msg>
using RclcppSingleThreadedExecutorCommunicator =
  RclcppCallbackCommunicator<Msg, rclcpp::executors::SingleThreadedExecutor>;

template<class Msg>
using RclcppStaticSingleThreadedExecutorCommunicator =
  RclcppCallbackCommunicator<Msg, rclcpp::executors::StaticSingleThreadedExecutor>;

}  // namespace performance_test
#endif  // COMMUNICATION_ABSTRACTIONS__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
