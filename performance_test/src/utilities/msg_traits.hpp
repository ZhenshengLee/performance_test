// Copyright 2022 Apex.AI, Inc.
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

#ifndef UTILITIES__MSG_TRAITS_HPP_
#define UTILITIES__MSG_TRAITS_HPP_

namespace performance_test
{
struct MsgTraits
{
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
  static inline std::enable_if_t<has_bounded_sequence<T>::value ||
    has_unbounded_sequence<T>::value ||
    has_unbounded_string<T>::value,
    void>
  ensure_fixed_size(T &)
  {
    throw std::runtime_error(
            "This plugin only supports messages with a fixed size");
  }

  template<typename T>
  static inline std::enable_if_t<!has_bounded_sequence<T>::value &&
    !has_unbounded_sequence<T>::value &&
    !has_unbounded_string<T>::value,
    void>
  ensure_fixed_size(T &) {}
};

}  // namespace performance_test

#endif  // UTILITIES__MSG_TRAITS_HPP_
