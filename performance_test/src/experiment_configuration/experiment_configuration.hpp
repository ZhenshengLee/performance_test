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

#ifndef EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
#define EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_

#include <atomic>
#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <chrono>

#include "qos_abstraction.hpp"
#include "communication_mean.hpp"
#include "execution_strategy.hpp"
#include "../outputs/output.hpp"

#if PERFORMANCE_TEST_RT_ENABLED
#include "../utilities/rt_enabler.hpp"
#endif
#include "external_info_storage.hpp"

namespace performance_test
{

/**
 * \brief Represents the configuration of an experiment.
 *
 * This experiment configuration could be created from various sources. At the
 * moment, only configuration by command line arguments are supported.
 */
class ExperimentConfiguration
{
public:
  // Implementing the standard C++11 singleton pattern.
  /// The singleton instance getter.
  static ExperimentConfiguration & get()
  {
    static ExperimentConfiguration instance;

    return instance;
  }

  ExperimentConfiguration(ExperimentConfiguration const &) = delete;
  ExperimentConfiguration(ExperimentConfiguration &&) = delete;

  ExperimentConfiguration & operator=(ExperimentConfiguration const &) = delete;

  ExperimentConfiguration & operator=(ExperimentConfiguration &&) = delete;

  enum RoundTripMode
  {
    NONE,  /// No roundtrip. Samples are only sent from sender to reciever.
    MAIN,  /// Sends packages to the relay and receives packages from the relay.
    RELAY  /// Relays packages from MAIN back to MAIN.
  };

  enum class SupportedOutput
  {
    STDOUT,  // print results to stdout in readable format
    CSV,     // print results to file in csv format
    JSON     // print results to file in json format
  };

  /// \brief Derives an experiment configuration from command line arguments.
  void setup(int argc, char ** argv);
  /// Returns if the experiment configuration is set up and ready to use.
  /// Note that all of the following getter functions will throw an exception
  /// if the ExperimentConfiguration is not set up.
  bool is_setup() const;

  CommunicationMean com_mean() const;
  ExecutionStrategy execution_strategy() const;
  /// \returns Returns whether the ROS 2 layers are used by the communication mean.
  bool use_ros2_layers() const;
  uint32_t dds_domain_id() const;
  QOSAbstraction qos() const;
  uint32_t rate() const;
  /// \returns Returns the inverse of the configured publishing rate.
  std::chrono::duration<double> period() const;
  /// \returns Returns the inverse of the configured publishing rate, in nanoseconds.
  std::chrono::nanoseconds period_ns() const;
  std::string topic_name() const;
  std::string msg_name() const;
  uint64_t max_runtime() const;
  uint32_t rows_to_ignore() const;
  uint32_t number_of_publishers() const;
  uint32_t number_of_subscribers() const;
  uint32_t expected_num_pubs() const;
  uint32_t expected_num_subs() const;
  std::chrono::seconds expected_wait_for_matched_timeout() const;
  bool check_memory() const;
  /// \returns Returns if post-proc RT initialization is required. This is set when the cpu
  /// affinity or thread priority is overridden by the caller.
  bool is_rt_init_required() const;
  bool is_with_security() const;
  /// \returns Returns whether shared memory transfer is enabled.
  /// Only implemented for Apex.OS.
  bool is_shared_memory_transfer() const;
  bool is_zero_copy_transfer() const;
  RoundTripMode roundtrip_mode() const;
  std::string rmw_implementation() const;
  std::string perf_test_version() const;
  std::string pub_topic_postfix() const;
  std::string sub_topic_postfix() const;
  /// \returns Returns the randomly generated unique ID of the experiment.
  std::string id() const;
  std::string logfile_name() const;
  size_t unbounded_msg_size() const;
  const std::vector<std::shared_ptr<Output>> & configured_outputs() const;
  /// \return Returns true if the user requested the application to exit.
  bool exit_requested() const;
  /// Request the application to exit.
  void request_exit();

  ExternalInfoStorage get_external_info() const
  {
    return m_external_info;
  }

private:
  ExperimentConfiguration();

  /// Throws std::runtime_error if the experiment is not set up.
  void check_setup() const;

  std::string m_id;
  bool m_is_setup;
  std::string m_logfile;
  std::string m_logfile_name;
  std::vector<std::shared_ptr<Output>> m_configured_outputs{};
  CommunicationMean m_com_mean;
  ExecutionStrategy m_execution_strategy;
  uint32_t m_dds_domain_id;
  QOSAbstraction m_qos;
  uint32_t m_rate;
  std::string m_topic_name;
  std::string m_msg_name;
  size_t m_unbounded_msg_size;

  uint64_t m_max_runtime;
  uint32_t m_rows_to_ignore;
  uint32_t m_number_of_publishers;
  uint32_t m_number_of_subscribers;
  uint32_t m_expected_num_pubs;
  uint32_t m_expected_num_subs;
  uint32_t m_wait_for_matched_timeout;
  bool m_check_memory;
  bool m_is_rt_init_required;
  bool m_with_security;
  bool m_is_zero_copy_transfer;
  std::atomic_bool m_exit_requested;

  RoundTripMode m_roundtrip_mode;

  std::string m_rmw_implementation;
  std::string m_perf_test_version;

  ExternalInfoStorage m_external_info;
};

std::string to_string(const ExperimentConfiguration::RoundTripMode e);
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration::RoundTripMode & e);
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e);
}  // namespace performance_test

#endif  // EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
