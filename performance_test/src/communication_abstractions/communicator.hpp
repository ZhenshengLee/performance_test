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
#include <stdexcept>
#include <vector>

#include "../experiment_metrics/subscriber_stats.hpp"
#include "../utilities/msg_traits.hpp"

namespace performance_test
{

class Publisher
{
public:
  virtual void publish_copy(std::int64_t time, std::uint64_t sample_id) = 0;

  virtual void publish_loaned(std::int64_t time, std::uint64_t sample_id) = 0;

protected:
  template<typename T>
  inline void init_msg(T & msg, std::int64_t time, std::uint64_t sample_id)
  {
    msg.time = time;
    msg.id = sample_id;
    MsgTraits::ensure_fixed_size(msg);
  }
};

class Subscriber
{
public:
  virtual std::vector<ReceivedMsgStats> update_subscription() = 0;

  virtual std::vector<ReceivedMsgStats> take()
  {
    throw std::runtime_error("This communicator does not support take!");
  }
};

}  // namespace performance_test

#endif  // COMMUNICATION_ABSTRACTIONS__COMMUNICATOR_HPP_
