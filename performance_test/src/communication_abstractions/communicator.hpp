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

#ifndef COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_
#define COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_

#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "../experiment_execution/data_stats.hpp"

namespace performance_test
{

/// Helper base class for communication plugins which provides sample tracking
/// helper functionality.
class Communicator
{
public:
  explicit Communicator(DataStats & stats);

  /**
   * \brief Reads received data from the middleware and compute some statistics
   */
  virtual void update_subscription() = 0;

  /**
   * \brief Publishes the provided data.
   *
   *  The first time this function is called it also creates the data writer.
   *  Further it updates all internal counters while running.
   * \param time The time to fill into the data field.
   */
  virtual void publish(std::int64_t time) = 0;

protected:
  DataStats & m_stats;
  /// The experiment configuration.
  const ExperimentConfiguration & m_ec;

  // TODO(erik.snider) switch to std::void_t when upgrading to C++17
  template<class ...>
  using void_t = void;

  template<typename T, typename = void>
  struct has_bounded_sequence : std::false_type {};

  template<typename T>
  struct has_bounded_sequence<
    T, void_t<decltype(std::declval<T>().bounded_sequence)>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_unbounded_sequence : std::false_type {};

  template<typename T>
  struct has_unbounded_sequence<
    T, void_t<decltype(std::declval<T>().unbounded_sequence)>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_unbounded_string : std::false_type {};

  template<typename T>
  struct has_unbounded_string<
    T, void_t<decltype(std::declval<T>().unbounded_string)>>
    : std::true_type {};

  template<typename T>
  inline void init_msg(T & msg, std::int64_t time)
  {
    msg.time = time;
    msg.id = m_stats.next_sample_id();
    ensure_fixed_size(msg);
  }

  template<typename T>
  inline std::enable_if_t<has_bounded_sequence<T>::value ||
    has_unbounded_sequence<T>::value ||
    has_unbounded_string<T>::value,
    void>
  ensure_fixed_size(T &)
  {
    throw std::runtime_error(
            "This plugin only supports messages with a fixed size");
  }

  template<typename T>
  inline std::enable_if_t<!has_bounded_sequence<T>::value &&
    !has_unbounded_sequence<T>::value &&
    !has_unbounded_string<T>::value,
    void>
  ensure_fixed_size(T &) {}
  void check_data_consistency(const std::int64_t time_ns_since_epoch);

private:
  /// The time the last sample was received [ns since epoc].
  std::int64_t m_prev_timestamp;
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_
