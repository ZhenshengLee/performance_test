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

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <string>
#include <fstream>
#include <vector>
#include <memory>

#include "qos_abstraction.hpp"
#include "communication_mean.hpp"
#if PERFORMANCE_TEST_RT_ENABLED
#include "../utilities/rt_enabler.hpp"
#endif
#include "external_info_storage.hpp"

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #include <odb/core.hxx>
#endif
#include <chrono>

namespace performance_test
{

/**
 * \brief Represents the configuration of an experiment.
 *
 * This experiment configuration could be created from various sources. At the moment, only
 * configuration by command line arguments are supported.
 */
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
#pragma db model version(1, 10, closed)
class AnalysisResult;
  #pragma db value(QOSAbstraction) definition
  #pragma db value(ExternalInfoStorage) definition
  #pragma db object
#endif
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
  /// \returns Returns if only a single participant should be used. This will throw if
  /// the experiment configuration is not set up.
  bool use_single_participant() const;
  /// \returns Returns if post-proc RT initialization is required. This is set when the cpu
  /// affinity or thread priority is overridden by the caller. This will throw if the experiment
  /// configuration is not set up.
  bool is_rt_init_required() const;
  /// \returns Returns if logging of performance_test results is disabled for stdout.
  /// This will throw if the experiment configuration is not set up.
  bool disable_logging() const;
  /// \returns Returns if security is enabled for ROS2. This will throw if the configured mean
  /// of communication is not ROS2.
  bool is_with_security() const;
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
  boost::uuids::uuid id() const;
  /// Logs \param msg to stdout and the configured log file. This will throw if the experiment
  /// configuration is not set up.
  void log(const std::string & msg) const;
  /// The configured logfile name. This will throw if the experiment configuration is not set up.
  std::string logfile_name() const;
  /// \return Returns true if the user requested the application to exit.
  bool exit_requested() const;
  ExternalInfoStorage get_external_info() const
  {
    return m_external_info;
  }
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  std::string db_name() const;
  bool use_odb() const;
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
  std::string db_user() const;
  std::string db_password() const;
  std::string db_host() const;
  unsigned int db_port() const;
#endif

  std::vector<std::shared_ptr<AnalysisResult>> & get_results() const
  {
    return m_results;
  }
#endif

private:
  ExperimentConfiguration()
  : m_id(boost::uuids::random_generator()()),
    m_is_setup(false),
    m_dds_domain_id(),
    m_rate(),
    m_max_runtime(),
    m_rows_to_ignore(),
    m_number_of_publishers(),
    m_number_of_subscribers(),
    m_expected_num_pubs(),
    m_expected_num_subs(),
    m_wait_for_matched_timeout(),
    m_check_memory(false),
    m_use_single_participant(false),
    m_is_rt_init_required(false),
    m_disable_logging(false),
    m_roundtrip_mode(RoundTripMode::NONE)
  {}

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  friend class odb::access;
#endif

  /// Throws #std::runtime_error if the experiment is not set up.
  void check_setup() const;

  /// Generates filename from the experiment configuration and opens a file accordingly. This will
  /// throw if the experiment configuration is not set up.
  void open_file();

  // Using the GUID of the experiment as ID.
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db id
#endif
  boost::uuids::uuid m_id;
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  bool m_is_setup;
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  std::string m_logfile;
  std::string m_final_logfile_name;

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  mutable std::ofstream m_os;

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  CommunicationMean m_com_mean;
  uint32_t m_dds_domain_id;
  QOSAbstraction m_qos;
  uint32_t m_rate;
  std::string m_topic_name;

  uint64_t m_max_runtime;
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  uint32_t m_rows_to_ignore;
  uint32_t m_number_of_publishers;
  uint32_t m_number_of_subscribers;
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  uint32_t m_expected_num_pubs;
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  uint32_t m_expected_num_subs;
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  uint32_t m_wait_for_matched_timeout;
  bool m_check_memory;
  bool m_use_single_participant;
  bool m_is_rt_init_required;
  bool m_with_security;
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db transient
#endif
  bool m_disable_logging;

  RoundTripMode m_roundtrip_mode;

  std::string m_rmw_implementation;
  std::string m_perf_test_version;

  ExternalInfoStorage m_external_info;

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  #pragma db value_not_null inverse(m_configuration)
  mutable std::vector<std::shared_ptr<AnalysisResult>> m_results;
  #pragma db transient
  bool m_use_odb = true;
  #pragma db transient
  std::string m_db_name;
  std::string m_com_mean_str;
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
  #pragma db transient
  std::string m_db_user;
  #pragma db transient
  std::string m_db_password;
  #pragma db transient
  std::string m_db_host;
  #pragma db transient
  unsigned int m_db_port;
#endif
#endif
};

/// Outstream operator for RoundTripMode.
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration::RoundTripMode & e);


/// Outstream operator for ExperimentConfiguration.
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e);
}  // namespace performance_test

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
#include "analysis_result.hpp"
#endif
#endif  // EXPERIMENT_CONFIGURATION__EXPERIMENT_CONFIGURATION_HPP_
