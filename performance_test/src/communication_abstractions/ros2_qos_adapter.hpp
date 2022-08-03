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

#ifndef COMMUNICATION_ABSTRACTIONS__ROS2_QOS_ADAPTER_HPP_
#define COMMUNICATION_ABSTRACTIONS__ROS2_QOS_ADAPTER_HPP_


#include <rclcpp/rclcpp.hpp>

#include "../experiment_configuration/qos_abstraction.hpp"

namespace performance_test
{

/// Translates abstract QOS settings to specific QOS settings for ROS 2.
class ROS2QOSAdapter
{
public:
  explicit ROS2QOSAdapter(const QOSAbstraction qos)
  : m_qos(qos) {}

  inline rclcpp::QoS get() const
  {
    rclcpp::QoS ros_qos(m_qos.history_depth);

    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      ros_qos.best_effort();
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      ros_qos.reliable();
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      ros_qos.durability_volatile();
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      ros_qos.transient_local();
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      ros_qos.keep_all();
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      ros_qos.keep_last(m_qos.history_depth);
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    return ros_qos;
  }

private:
  const QOSAbstraction m_qos;
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__ROS2_QOS_ADAPTER_HPP_
