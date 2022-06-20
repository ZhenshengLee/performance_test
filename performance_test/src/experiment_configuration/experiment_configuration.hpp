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

#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <chrono>

#include "qos_abstraction.hpp"
#include "communication_mean.hpp"
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

  /// Specifies the selected roundtrip mode.
  enum RoundTripMode
  {
    NONE,  /// No roundtrip. Samples are only sent from sender to reciever.
    MAIN,  /// Sends packages to the relay and receives packages from the relay.
    RELAY  /// Relays packages from MAIN back to MAIN.
  };

  /// Specifies the supported output implementations.
  enum class SupportedOutput
  {
    STDOUT,  // print results to stdout in readable format
    CSV,     // print results to file in csv format
    JSON     // print results to file in json format
  };

  /**
   * \brief Derives an experiment configuration from command line arguments.
   * \param argc The argc parameter from the main function.
   * \param argv The argv parameter from the main function.
   */
  void setup(int argc, char ** argv);
  /// Returns if the experiment configuration is set up and ready to use.
  bool is_setup() const;

  /// \returns Returns the configured mean of communication. This will throw if the experiment
  /// configuration is not set up.
  CommunicationMean com_mean() const;
  /// \returns Returns whether the ROS 2 layers are used by the communication mean.
  bool use_ros2_layers() const;
  /// \returns Returns the configured DDS domain ID. This will throw if the experiment
  /// configuration is not set up.
  uint32_t dds_domain_id() const;
  /// \returns Returns the configured QOS settings. This will throw if the experiment
  /// configuration is not set up.
  QOSAbstraction qos() const;
  /// \returns Returns the configured publishing rate. This will throw if the experiment
  /// configuration is not set up.
  uint32_t rate() const;
  /// \returns Returns the chosen topic name. This will throw if the experiment configuration is
  /// not set up.
  std::string topic_name() const;
  /// \returns Returns the chosen msg type name. This will throw if the experiment configuration is
  /// not set up.
  std::string msg_name() const;
  /// \returns Returns the time the application should run until it terminates [s]. This will
  /// throw if the experiment configuration is not set up.
  uint64_t max_runtime() const;
  /// \returns Returns the number of seconds to be ignored at the beginning of the experiment.
  /// This will throw if the experiment configuration is not set up.
  uint32_t rows_to_ignore() const;
  /// \returns Returns the configured number of publishers. This will throw if the experiment
  /// configuration is not set up.
  uint32_t number_of_publishers() const;
  /// \returns Returns the configured number of subscribers. This will throw if the experiment
  /// configuration is not set up.
  uint32_t number_of_subscribers() const;
  /// \returns Returns the expected number of publishers for wait_for_matched if enabled. This
  /// will throw if the experiment configuration is not set up.
  uint32_t expected_num_pubs() const;
  /// \returns Returns the expected number of subscribers for wait_for_matched if enabled. This
  /// will throw if the experiment configuration is not set up.
  uint32_t expected_num_subs() const;
  /// \returns Returns the expected timeout for wait_for_matched if enabled. This
  /// will throw if the experiment configuration is not set up.
  std::chrono::seconds expected_wait_for_matched_timeout() const;
  /// \returns Returns if memory operations should be logged.
  bool check_memory() const;
  /// \returns Returns if post-proc RT initialization is required. This is set when the cpu
  /// affinity or thread priority is overridden by the caller. This will throw if the experiment
  /// configuration is not set up.
  bool is_rt_init_required() const;
  /// \returns Returns if security is enabled for ROS2. This will throw if the configured mean
  /// of communication is not ROS2.
  bool is_with_security() const;
  /// \returns Returns whether shared memory transfer is enabled. Only applicable for Apex.OS or
  /// ROS2. This will throw if the experiment configuration is not set up.
  bool is_shared_memory_transfer() const;
  /// \returns Returns whether to use zero copy transfer. This will throw if the experiment
  /// configuration is not set up.
  bool is_zero_copy_transfer() const;
  /// \returns Returns the roundtrip mode.
  RoundTripMode roundtrip_mode() const;
  /// \returns Returns current rmw_implementation. This will throw if the experiment configuration
  /// is not set up.
  std::string rmw_implementation() const;
  /// \returns Returns current performance test version. This will throw if the experiment
  /// configuration is not set up.
  std::string perf_test_version() const;
  /// \returns Returns the publishing topic postfix.
  std::string pub_topic_postfix() const;
  /// \returns Returns the subscribing topic postfix.
  std::string sub_topic_postfix() const;
  /// \returns Returns the randomly generated unique ID of the experiment. This will throw if the
  /// experiment configuration is not set up.
  std::string id() const;
  /// The configured logfile name. This will throw if the experiment configuration is not set up.
  std::string logfile_name() const;
  /// The number of bytes to use for an unbounded message.
  /// This will throw if the experiment configuration is not set up.
  size_t unbounded_msg_size() const;
  const std::vector<std::shared_ptr<Output>> & configured_outputs() const;
  /// \return Returns true if the user requested the application to exit.
  bool exit_requested() const;
  ExternalInfoStorage get_external_info() const
  {
    return m_external_info;
  }

private:
  ExperimentConfiguration();

  /// Throws #std::runtime_error if the experiment is not set up.
  void check_setup() const;

  // Using the GUID of the experiment as ID.
  std::string m_id;
  bool m_is_setup;
  std::string m_logfile;
  std::string m_logfile_name;
  std::vector<std::shared_ptr<Output>> m_configured_outputs{};
  CommunicationMean m_com_mean;
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

  RoundTripMode m_roundtrip_mode;

  std::string m_rmw_implementation;
  std::string m_perf_test_version;

  ExternalInfoStorage m_external_info;
};

std::string to_string(const ExperimentConfiguration::RoundTripMode e);
/// Outstream operator for RoundTripMode.
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration::RoundTripMode & e);

/// Outstream operator for ExperimentConfiguration.
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e);
}  // namespace performance_test

#endif  // EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
