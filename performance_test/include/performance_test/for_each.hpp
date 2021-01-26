// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef PERFORMANCE_TEST__FOR_EACH_HPP_
#define PERFORMANCE_TEST__FOR_EACH_HPP_

#include <tuple>
#include <utility>

namespace performance_test
{

/// Iterates over std::tuple types
/**
 * \param t tuple to be iterated
 * \param f functor to be applied over each element
 * \returns number of iterated elements
 */
template<std::size_t I = 0, typename FuncT, typename ... Tp>
typename std::enable_if<I == sizeof...(Tp), size_t>::type
for_each(const std::tuple<Tp...> & t, FuncT f)
{
  (void)t;
  (void)f;
  return I;
}

template<std::size_t I = 0, typename FuncT, typename ... Tp>
typename std::enable_if<I != sizeof...(Tp), size_t>::type
for_each(const std::tuple<Tp...> & t, FuncT f)
{
  f(std::get<I>(t));
  return for_each<I + 1, FuncT, Tp...>(t, f);
}

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__FOR_EACH_HPP_
