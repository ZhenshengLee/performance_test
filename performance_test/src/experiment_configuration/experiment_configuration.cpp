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

#include "experiment_configuration.hpp"

#include <boost/program_options.hpp>
#include <rmw/rmw.h>

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>

#include "topics.hpp"

#include "performance_test/version.h"
#include <rclcpp/rclcpp.hpp> // NOLINT - This include order is required when using OpenDDS

namespace performance_test
{

std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration::RoundTripMode & e)
{
  if (e == ExperimentConfiguration::RoundTripMode::NONE) {
    stream << "NONE";
  } else if (e == ExperimentConfiguration::RoundTripMode::MAIN) {
    stream << "MAIN";
  } else if (e == ExperimentConfiguration::RoundTripMode::RELAY) {
    stream << "RELAY";
  }
  return stream;
}
std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration & e)
{
  if (e.is_setup()) {
    return stream <<
           "Experiment id: " << e.id() <<
           "\nPerformance Test Version: " << e.perf_test_version() <<
           "\nLogfile name: " << e.logfile_name() <<
           "\nCommunication mean: " << e.com_mean() <<
           "\nRMW Implementation: " << e.rmw_implementation() <<
           "\nDDS domain id: " << e.dds_domain_id() <<
           "\nQOS: " << e.qos() <<
           "\nPublishing rate: " << e.rate() <<
           "\nTopic name: " << e.topic_name() <<
           "\nMaximum runtime (sec): " << e.max_runtime() <<
           "\nNumber of publishers: " << e.number_of_publishers() <<
           "\nNumber of subscribers: " << e.number_of_subscribers() <<
           "\nMemory check enabled: " << e.check_memory() <<
           "\nUse single participant: " << e.use_single_participant() <<
           "\nWith security: " << e.is_with_security() <<
           "\nRoundtrip Mode: " << e.roundtrip_mode() <<
           "\nIgnore seconds from beginning: " << e.rows_to_ignore();
  } else {
    return stream << "ERROR: Experiment is not yet setup!";
  }
}

void ExperimentConfiguration::setup(int argc, char ** argv)
{
  namespace po = ::boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "Print usage message.")(
    "logfile,l", po::value<std::string>(),
    "Optionally specify a logfile.")(
    "rate,r", po::value<uint32_t>()->default_value(1000),
    "The rate data should be published. Defaults to 1000 Hz. 0 means publish as fast as possible.")(
    "communication,c", po::value<std::string>()->required(),
    "Communication plugin to use (ROS2, FastRTPS, ConnextDDSMicro, CycloneDDS, OpenDDS, "
    "ROS2PollingSubscription)")(
    "topic,t",
    po::value<std::string>()->required(),
    "Topic to use. Use --topic_list to get a list.")(
    "topic_list",
    "Prints list of available topics and exits.")(
    "dds_domain_id",
    po::value<uint32_t>()->default_value(0), "Sets the DDS domain id.")(
    "reliable",
    "Enable reliable QOS. Default is best effort.")(
    "transient",
    "Enable transient QOS. Default is volatile.")(
    "keep_last",
    "Enable keep last QOS. Default is keep all.")(
    "history_depth",
    po::value<uint32_t>()->default_value(1000),
    "Set history depth QOS. Defaults to 1000.")(
    "disable_async",
    "Disables async. pub/sub.")(
    "max_runtime",
    po::value<uint64_t>()->default_value(0),
    "Maximum number of seconds to run before exiting. Default (0) is to run forever.")(
    "num_pub_threads,p", po::value<uint32_t>()->default_value(1),
    "Maximum number of publisher threads.")(
    "num_sub_threads,s",
    po::value<uint32_t>()->default_value(1),
    "Maximum number of subscriber threads.")(
    "check_memory",
    "Prints backtrace of all memory operations performed by the middleware. "
    "This will slow down the application!")(
    "use_rt_prio", po::value<int32_t>()->default_value(0),
    "Set RT priority. "
    "Only certain platforms (i.e. Drive PX) have the right configuration to support this.")(
    "use_rt_cpus", po::value<uint32_t>()->default_value(0),
    "Set RT cpu affinity mask. "
    "Only certain platforms (i.e. Drive PX) have the right configuration to support this.")(
    "use_single_participant",
    "Uses only one participant per process. By default every thread has its own.")(
    "with_security", "Make nodes with deterministic names for use with security")(
    "roundtrip_mode",
    po::value<std::string>()->default_value("None"),
    "Selects the round trip mode (None, Main, Relay).")(
    "ignore",
    po::value<uint32_t>()->default_value(0),
    "Ignores first n seconds of the experiment.")(
    "disable_logging",
    "Disables experiment logging to stdout.")(
    "expected_num_pubs",
    po::value<uint32_t>()->default_value(0), "Expected number of publishers for "
    "wait_for_matched")(
    "expected_num_subs",
    po::value<uint32_t>()->default_value(0), "Expected number of subscribers for "
    "wait_for_matched")(
    "wait_for_matched_timeout",
    po::value<uint32_t>()->default_value(30),
    "Maximum time[s] to wait for matching publishers/subscribers. Defaults to 30s")
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
  ("db_name", po::value<std::string>()->default_value("db_name"),
  "Name of the SQL database.")
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
  ("db_user", po::value<std::string>(),
  "User name to login to the SQL database.")(
    "db_password",
    po::value<std::string>(),
    "Password to login to the SQL database.")(
    "db_host",
    po::value<std::string>(), "IP address of SQL server.")(
    "db_port",
    po::value<unsigned int>(), "Port for SQL protocol.")
#endif
#endif
  ;
  po::variables_map vm;
#if defined(USE_LEGACY_QOS_API)
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
#else
  // ROS eloquent adds by default --ros-args to ros2 launch and no value for that argument
  // is valid, so we allow unregistered options so boost doesn't complain about it.
  po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
#endif
  m_perf_test_version = version;

  try {
    if (vm.count("topic_list")) {
      for (const auto & s : topics::supported_topic_names()) {
        std::cout << s << std::endl;
      }
      // Exiting as we just print out some information and not running the application.
      exit(0);
    }

    if (vm.count("help")) {
      std::cout << "Version: " << perf_test_version() << "\n";
      std::cout << desc << "\n";
      exit(0);
    }

    // validate arguments and raise an error if invalid
    po::notify(vm);

    m_topic_name = vm["topic"].as<std::string>();

    m_rate = vm["rate"].as<uint32_t>();

    if (vm.count("check_memory")) {
      m_check_memory = true;
    }

    if (vm["communication"].as<std::string>() == "ROS2") {
      m_com_mean = CommunicationMean::ROS2;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "ROS2";
      #endif
    } else if (vm["communication"].as<std::string>() == "ROS2PollingSubscription") {
#ifdef PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED
      m_com_mean = CommunicationMean::ROS2PollingSubscription;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "ROS2PollingSubscription";
      #endif
#else
      throw std::invalid_argument(
              "You must compile with PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED flag as ON to "
              "enable it as communication mean.");
#endif
    } else if (vm["communication"].as<std::string>() == "FastRTPS") {
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
      m_com_mean = CommunicationMean::FASTRTPS;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "FASTRTPS";
      #endif
#else
      throw std::invalid_argument(
              "You must compile with FastRTPS support to enable it as communication mean.");
#endif
    } else if (vm["communication"].as<std::string>() == "ConnextDDSMicro") {
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
      m_com_mean = CommunicationMean::CONNEXTDDSMICRO;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "CONNEXTDDSMICRO";
      #endif
#else
      throw std::invalid_argument(
              "You must compile with ConnextDDSMicro support to enable it as communication mean.");
#endif
    } else if (vm["communication"].as<std::string>() == "CycloneDDS") {
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
      m_com_mean = CommunicationMean::CYCLONEDDS;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "CYCLONEDDS";
      #endif
#else
      throw std::invalid_argument(
              "You must compile with CycloneDDS support to enable it as communication mean.");
#endif
    } else if (vm["communication"].as<std::string>() == "OpenDDS") {
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
      m_com_mean = CommunicationMean::OPENDDS;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "OPENDDS";
      #endif
#else
      throw std::invalid_argument(
              "You must compile with OpenDDS support to enable it as communication mean");
#endif
    } else {
      throw std::invalid_argument("Selected communication mean not supported!");
    }

    m_dds_domain_id = vm["dds_domain_id"].as<uint32_t>();

    if (vm.count("reliable")) {
      m_qos.reliability = QOSAbstraction::Reliability::RELIABLE;
    } else {
      m_qos.reliability = QOSAbstraction::Reliability::BEST_EFFORT;
    }
    if (vm.count("transient")) {
      m_qos.durability = QOSAbstraction::Durability::TRANSIENT_LOCAL;
    } else {
      m_qos.durability = QOSAbstraction::Durability::VOLATILE;
    }
    if (vm.count("keep_last")) {
      m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_LAST;
    } else {
      m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_ALL;
    }
    m_qos.history_depth = vm["history_depth"].as<uint32_t>();
    if (vm.count("disable_async")) {
      if (m_com_mean == CommunicationMean::ROS2) {
        throw std::invalid_argument("ROS 2 does not support disabling async. publishing.");
      }
      m_qos.sync_pubsub = true;
    }

    m_max_runtime = vm["max_runtime"].as<uint64_t>();
    m_rows_to_ignore = vm["ignore"].as<uint32_t>();

    m_number_of_publishers = vm["num_pub_threads"].as<uint32_t>();
    m_number_of_subscribers = vm["num_sub_threads"].as<uint32_t>();

    if (m_number_of_publishers > 1) {
      throw std::invalid_argument("More than one publisher is not supported at the moment");
    }

    m_expected_num_pubs = vm["expected_num_pubs"].as<uint32_t>();
    m_expected_num_subs = vm["expected_num_subs"].as<uint32_t>();
    m_wait_for_matched_timeout = vm["wait_for_matched_timeout"].as<uint32_t>();

    if (m_expected_num_pubs > 1) {
      throw std::invalid_argument("More than one publisher is not supported at the moment");
    }

    int32_t prio = vm["use_rt_prio"].as<int32_t>();
    uint32_t cpus = vm["use_rt_cpus"].as<uint32_t>();

    if (prio != 0 || cpus != 0) {
#if PERFORMANCE_TEST_RT_ENABLED
      pre_proc_rt_init(cpus, prio);
      m_is_rt_init_required = true;
#else
      throw std::invalid_argument("Built with RT optimizations disabled");
#endif
    }
    m_use_single_participant = false;
    if (vm.count("use_single_participant")) {
      if (m_com_mean == CommunicationMean::ROS2) {
        throw std::invalid_argument("ROS2 does not support single participant mode!");
      } else {
        m_use_single_participant = true;
      }
    }

    m_with_security = false;
    if (vm.count("with_security")) {
      if (m_com_mean != CommunicationMean::ROS2) {
        throw std::invalid_argument("Only ROS2 supports security!");
      } else {
        m_with_security = true;
      }
    }
    m_roundtrip_mode = RoundTripMode::NONE;
    const auto mode = vm["roundtrip_mode"].as<std::string>();
    if (mode == "None") {
      m_roundtrip_mode = RoundTripMode::NONE;
    } else if (mode == "Main") {
      m_roundtrip_mode = RoundTripMode::MAIN;
    } else if (mode == "Relay") {
      m_roundtrip_mode = RoundTripMode::RELAY;
    } else {
      throw std::invalid_argument("Invalid roundtrip mode: " + mode);
    }
    m_rmw_implementation = rmw_get_implementation_identifier();

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
    m_db_name = vm["db_name"].as<std::string>();
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
    if (vm.count("db_user")) {
      m_db_user = vm["db_user"].as<std::string>();
    }
    if (vm.count("db_password")) {
      m_db_password = vm["db_password"].as<std::string>();
    }
    if (vm.count("db_host")) {
      m_db_host = vm["db_host"].as<std::string>();
    }
    if (vm.count("db_port")) {
      m_db_port = vm["db_port"].as<unsigned int>();
    }
    if (!vm.count("db_user") || !vm.count("db_password") || !vm.count("db_host") ||
      !vm.count("db_port"))
    {
      m_use_odb = false;
      std::cout <<
        "Required database information not provided, running the experiment without SQL support!" <<
        std::endl;
    }
#endif
#endif
    m_is_setup = true;
    // Logfile needs to be opened at the end, as the experiment configuration influences the
    // filename.
    if (vm.count("logfile")) {
      m_logfile = vm["logfile"].as<std::string>();
      open_file();
    }
    // If we need to disable logging to stdout
    if (vm.count("disable_logging")) {
      m_disable_logging = true;
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR: ";
    std::cerr << e.what() << std::endl;
    std::cerr << "Check below on how to use this tool:" << std::endl;
    std::cerr << desc << "\n";
    exit(1);
  }
}

bool ExperimentConfiguration::is_setup() const
{
  return m_is_setup;
}
CommunicationMean ExperimentConfiguration::com_mean() const
{
  check_setup();
  return m_com_mean;
}
uint32_t ExperimentConfiguration::dds_domain_id() const
{
  check_setup();
  return m_dds_domain_id;
}
QOSAbstraction ExperimentConfiguration::qos() const
{
  check_setup();
  return m_qos;
}
uint32_t ExperimentConfiguration::rate() const
{
  check_setup();
  return m_rate;
}
std::string ExperimentConfiguration::topic_name() const
{
  check_setup();
  return m_topic_name;
}
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
std::string ExperimentConfiguration::db_name() const
{
  check_setup();
  return m_db_name;
}
bool ExperimentConfiguration::use_odb() const
{
  return m_use_odb;
}
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
std::string ExperimentConfiguration::db_user() const
{
  return m_db_user;
}
std::string ExperimentConfiguration::db_password() const
{
  return m_db_password;
}
std::string ExperimentConfiguration::db_host() const
{
  return m_db_host;
}
unsigned int ExperimentConfiguration::db_port() const
{
  return m_db_port;
}
#endif
#endif
uint64_t ExperimentConfiguration::max_runtime() const
{
  check_setup();
  return m_max_runtime;
}
uint32_t ExperimentConfiguration::rows_to_ignore() const
{
  check_setup();
  return m_rows_to_ignore;
}
uint32_t ExperimentConfiguration::number_of_publishers() const
{
  check_setup();
  return m_number_of_publishers;
}
uint32_t ExperimentConfiguration::number_of_subscribers() const
{
  check_setup();
  return m_number_of_subscribers;
}

uint32_t ExperimentConfiguration::expected_num_pubs() const
{
  check_setup();
  return m_expected_num_pubs;
}
uint32_t ExperimentConfiguration::expected_num_subs() const
{
  check_setup();
  return m_expected_num_subs;
}

std::chrono::seconds ExperimentConfiguration::expected_wait_for_matched_timeout() const
{
  check_setup();
  return std::chrono::seconds(m_wait_for_matched_timeout);
}

bool ExperimentConfiguration::check_memory() const
{
  check_setup();
  return m_check_memory;
}

bool ExperimentConfiguration::use_single_participant() const
{
  check_setup();
  return m_use_single_participant;
}

bool ExperimentConfiguration::is_rt_init_required() const
{
  check_setup();
  return m_is_rt_init_required;
}

bool ExperimentConfiguration::is_with_security() const
{
  check_setup();
  return m_with_security;
}

bool ExperimentConfiguration::disable_logging() const
{
  check_setup();
  return m_disable_logging;
}

ExperimentConfiguration::RoundTripMode ExperimentConfiguration::roundtrip_mode() const
{
  check_setup();
  return m_roundtrip_mode;
}

std::string ExperimentConfiguration::rmw_implementation() const
{
  check_setup();
  return m_rmw_implementation;
}

std::string ExperimentConfiguration::perf_test_version() const
{
  return m_perf_test_version;
}

std::string ExperimentConfiguration::pub_topic_postfix() const
{
  check_setup();
  std::string fix;
  if (m_roundtrip_mode == ExperimentConfiguration::RoundTripMode::MAIN) {
    fix = "main";
  } else if (m_roundtrip_mode == ExperimentConfiguration::RoundTripMode::RELAY) {
    fix = "relay";
  }
  return fix;
}

std::string ExperimentConfiguration::sub_topic_postfix() const
{
  check_setup();
  std::string fix;
  if (m_roundtrip_mode == ExperimentConfiguration::RoundTripMode::MAIN) {
    fix = "relay";
  } else if (m_roundtrip_mode == ExperimentConfiguration::RoundTripMode::RELAY) {
    fix = "main";
  }
  return fix;
}

boost::uuids::uuid ExperimentConfiguration::id() const
{
  return m_id;
}

void ExperimentConfiguration::log(const std::string & msg) const
{
  if (!m_disable_logging) {
    std::cout << msg << std::endl;
  }
  if (m_os.is_open()) {
    m_os << msg << std::endl;
  }
}

std::string ExperimentConfiguration::logfile_name() const
{
  return m_final_logfile_name;
}

void ExperimentConfiguration::check_setup() const
{
  if (!m_is_setup) {
    throw std::runtime_error("Experiment is not yet setup!");
  }
}

void ExperimentConfiguration::open_file()
{
  check_setup();
  auto t = std::time(nullptr);
  auto tm = *std::gmtime(&t);
  std::ostringstream oss;
  oss << m_logfile.c_str() << "_" << m_topic_name << std::put_time(&tm, "_%d-%m-%Y_%H-%M-%S");
  m_final_logfile_name = oss.str();
  m_os.open(m_final_logfile_name, std::ofstream::out);
}

bool ExperimentConfiguration::exit_requested() const
{
  return !rclcpp::ok();
}

}  // namespace performance_test
