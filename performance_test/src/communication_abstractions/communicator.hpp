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
#include "../utilities/msg_traits.hpp"

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
  const ExperimentConfiguration & m_ec;

  template<typename T>
  inline void init_msg(T & msg, std::int64_t time)
  {
    msg.time = time;
    msg.id = m_stats.next_sample_id();
    MsgTraits::ensure_fixed_size(msg);
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_
