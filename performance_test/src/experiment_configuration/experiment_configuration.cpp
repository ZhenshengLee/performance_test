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

#include <tclap/CmdLine.h>
#include <rmw/rmw.h>

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <vector>

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
           "\nMsg name: " << e.msg_name() <<
           "\nMaximum runtime (sec): " << e.max_runtime() <<
           "\nNumber of publishers: " << e.number_of_publishers() <<
           "\nNumber of subscribers: " << e.number_of_subscribers() <<
           "\nMemory check enabled: " << e.check_memory() <<
           "\nUse single participant: " << e.use_single_participant() <<
           "\nWith security: " << e.is_with_security() <<
           "\nZero copy transfer: " << e.is_zero_copy_transfer() <<
           "\nRoundtrip Mode: " << e.roundtrip_mode() <<
           "\nIgnore seconds from beginning: " << e.rows_to_ignore();
  } else {
    return stream << "ERROR: Experiment is not yet setup!";
  }
}

void ExperimentConfiguration::setup(int argc, char ** argv)
{
  std::string comm_str;
  bool print_msg_list = false;
  bool reliable_qos = false;
  bool transient_qos = false;
  bool keep_last_qos = false;
  uint32_t history_depth = 0;
  bool disable_async = false;
  int32_t prio = 0;
  uint32_t cpus = 0;
  std::string roundtrip_mode_str;
  try {
    TCLAP::CmdLine cmd("Apex.AI performance_test");

    TCLAP::ValueArg<std::string> logfileArg("l", "logfile",
      "Optionally specify a logfile.", false, "", "name", cmd);

    TCLAP::ValueArg<uint32_t> rateArg("r", "rate",
      "The publishing rate. 0 means publish as fast as possible.", false, 1000, "N", cmd);

    std::vector<std::string> allowedCommunications;
#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
    allowedCommunications.push_back("rclcpp-single-threaded-executor");
    allowedCommunications.push_back("rclcpp-static-single-threaded-executor");
    allowedCommunications.push_back("rclcpp-waitset");
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    allowedCommunications.push_back("ApexOSPollingSubscription");
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
    allowedCommunications.push_back("FastRTPS");
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
    allowedCommunications.push_back("ConnextDDSMicro");
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
    allowedCommunications.push_back("ConnextDDS");
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
    allowedCommunications.push_back("CycloneDDS");
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
    allowedCommunications.push_back("iceoryx");
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
    allowedCommunications.push_back("OpenDDS");
#endif
    TCLAP::ValuesConstraint<std::string> allowedCommunicationVals(allowedCommunications);
    TCLAP::ValueArg<std::string> communicationArg("c", "communication",
      "Which communication plugin to use.", false, allowedCommunications[0],
      &allowedCommunicationVals, cmd);

    TCLAP::ValueArg<std::string> topicArg("t", "topic",
      "The topic name.", false, "test_topic", "topic", cmd);

    TCLAP::ValueArg<std::string> msgArg("m", "msg",
      "The message type. Use --msg-list to list the options.", false, "Array1k", "type", cmd);

    TCLAP::SwitchArg msgListArg("", "msg-list",
      "Print the list of available msg types and exit.", cmd, false);

    TCLAP::ValueArg<uint32_t> ddsDomainIdArg("", "dds-domain_id",
      "The DDS domain id.", false, 0, "id", cmd);

    TCLAP::SwitchArg reliableArg("", "reliable",
      "Enable reliable QOS. Default is best effort.", cmd, false);

    TCLAP::SwitchArg transientArg("", "transient",
      "Enable transient local QOS. Default is volatile.", cmd, false);

    TCLAP::SwitchArg keepLastArg("", "keep-last",
      "Enable keep last QOS. Default is keep all.", cmd, false);

    TCLAP::ValueArg<uint32_t> historyDepthArg("", "history-depth",
      "The history depth QOS.", false, 1000, "N", cmd);

    TCLAP::SwitchArg disableAsyncArg("", "disable-async",
      "Disable asynchronous pub/sub.", cmd, false);

    TCLAP::ValueArg<uint64_t> maxRuntimeArg("", "max-runtime",
      "Run N seconds, then exit. 0 means run forever.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> numPubsArg("p", "num-pub-threads",
      "Number of publisher threads.", false, 1, "N", cmd);

    TCLAP::ValueArg<uint32_t> numSubsArg("s", "num-sub-threads",
      "Number of subscriber threads.", false, 1, "N", cmd);

    TCLAP::SwitchArg checkMemoryArg("", "check-memory",
      "Print backtrace of all memory operations performed by the middleware. "
      "This will slow down the application!", cmd, false);

    TCLAP::ValueArg<int32_t> useRtPrioArg("", "use-rt-prio",
      "Set RT priority. "
      "Only certain platforms (i.e. Drive PX) have the right configuration to support this.",
      false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> useRtCpusArg("", "use-rt-cpus",
      "Set RT CPU affinity mask. "
      "Only certain platforms (i.e. Drive PX) have the right configuration to support this.",
      false, 0, "N", cmd);

    TCLAP::SwitchArg useSingleParticipantArg("", "use-single-participant",
      "**DEPRECATED** Uses only one participant per process. By default every thread has its own.",
      cmd, false);

    TCLAP::SwitchArg withSecurityArg("", "with-security",
      "Make nodes with deterministic names for use with security.", cmd, false);

    std::vector<std::string> allowedRelayModes{{"None", "Main", "Relay"}};
    TCLAP::ValuesConstraint<std::string> allowedRelayModeVals(allowedRelayModes);
    TCLAP::ValueArg<std::string> relayModeArg("", "roundtrip-mode",
      "Select the round trip mode.", false, "None",
      &allowedRelayModeVals, cmd);

    TCLAP::ValueArg<uint32_t> ignoreArg("", "ignore",
      "Ignore the first N seconds of the experiment.", false, 0, "N", cmd);

    TCLAP::SwitchArg disableLoggingArg("", "disable-logging",
      "Disable experiment logging to stdout.", cmd, false);

    TCLAP::ValueArg<uint32_t> expectedNumPubsArg("", "expected-num-pubs",
      "Expected number of publishers for wait-for-matched.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> expectedNumSubsArg("", "expected-num-subs",
      "Expected number of subscribers for wait-for-matched.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> waitForMatchedTimeoutArg("", "wait-for-matched-timeout",
      "Maximum time in seconds to wait for matched pubs/subs.", false, 30, "N", cmd);

    TCLAP::SwitchArg zeroCopyArg("", "zero-copy",
      "Use zero copy transfer.", cmd, false);

#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
    TCLAP::ValueArg<std::string> dbNameArg("", "db-name",
      "Name of the SQL database.", false, "db-name", "db", cmd);
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
    TCLAP::ValueArg<std::string> dbUserArg("", "db-user",
      "User name to login to the SQL database.", false, "", "user", cmd);

    TCLAP::ValueArg<std::string> dbPasswordArg("", "db-password",
      "Password to login to the SQL database.", false, "", "pw", cmd);

    TCLAP::ValueArg<std::string> dbHostArg("", "db-host",
      "IP address of the SQL server.", false, "", "host", cmd);

    TCLAP::ValueArg<unsigned int> dbPortArg("", "db-port",
      "Port for SQL protocol.", false, 0, "port", cmd);
#endif
#endif

    cmd.parse(argc, argv);

    m_logfile = logfileArg.getValue();
    m_rate = rateArg.getValue();
    comm_str = communicationArg.getValue();
    m_topic_name = topicArg.getValue();
    m_msg_name = msgArg.getValue();
    print_msg_list = msgListArg.getValue();
    m_dds_domain_id = ddsDomainIdArg.getValue();
    reliable_qos = reliableArg.getValue();
    transient_qos = transientArg.getValue();
    keep_last_qos = keepLastArg.getValue();
    history_depth = historyDepthArg.getValue();
    disable_async = disableAsyncArg.getValue();
    m_max_runtime = maxRuntimeArg.getValue();
    m_number_of_publishers = numPubsArg.getValue();
    m_number_of_subscribers = numSubsArg.getValue();
    m_check_memory = checkMemoryArg.getValue();
    prio = useRtPrioArg.getValue();
    cpus = useRtCpusArg.getValue();
    m_use_single_participant = useSingleParticipantArg.getValue();
    m_with_security = withSecurityArg.getValue();
    roundtrip_mode_str = relayModeArg.getValue();
    m_rows_to_ignore = ignoreArg.getValue();
    m_disable_logging = disableLoggingArg.getValue();
    m_expected_num_pubs = expectedNumPubsArg.getValue();
    m_expected_num_subs = expectedNumSubsArg.getValue();
    m_wait_for_matched_timeout = waitForMatchedTimeoutArg.getValue();
    m_is_zero_copy_transfer = zeroCopyArg.getValue();
#ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
    m_db_name = dbNameArg.getValue();
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
    m_db_user = dbUserArg.getValue();
    m_db_password = dbPasswordArg.getValue();
    m_db_host = dbHostArg.getValue();
    m_db_port = dbPortArg.getValue();
#endif
#endif
  } catch (TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  m_perf_test_version = version;

  try {
    if (print_msg_list) {
      for (const auto & s : topics::supported_msg_names()) {
        std::cout << s << std::endl;
      }
      // Exiting as we just print out some information and not running the application.
      exit(0);
    }

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
    if (comm_str == "rclcpp-single-threaded-executor") {
      m_com_mean = CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "RCLCPP_SINGLE_THREADED_EXECUTOR";
      #endif
    }
    if (comm_str == "rclcpp-static-single-threaded-executor") {
      m_com_mean = CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR";
      #endif
    }
    if (comm_str == "rclcpp-waitset") {
      m_com_mean = CommunicationMean::RCLCPP_WAITSET;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "RCLCPP_WAITSET";
      #endif
    }
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    if (comm_str == "ApexOSPollingSubscription") {
      m_com_mean = CommunicationMean::ApexOSPollingSubscription;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "ApexOSPollingSubscription";
      #endif
    }
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
    if (comm_str == "FastRTPS") {
      m_com_mean = CommunicationMean::FASTRTPS;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "FASTRTPS";
      #endif
    }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
    if (comm_str == "ConnextDDSMicro") {
      m_com_mean = CommunicationMean::CONNEXTDDSMICRO;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "CONNEXTDDSMICRO";
      #endif
    }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
    if (comm_str == "ConnextDDS") {
      m_com_mean = CommunicationMean::CONNEXTDDS;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "CONNEXTDDS";
      #endif
    }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
    if (comm_str == "CycloneDDS") {
      m_com_mean = CommunicationMean::CYCLONEDDS;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "CYCLONEDDS";
      #endif
    }
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
    if (comm_str == "iceoryx") {
      m_com_mean = CommunicationMean::ICEORYX;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "ICEORYX";
      #endif
    }
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
    if (comm_str == "OpenDDS") {
      m_com_mean = CommunicationMean::OPENDDS;
      #ifdef PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED
      m_com_mean_str = "OPENDDS";
      #endif
    }
#endif

    if (reliable_qos) {
      m_qos.reliability = QOSAbstraction::Reliability::RELIABLE;
    } else {
      m_qos.reliability = QOSAbstraction::Reliability::BEST_EFFORT;
    }
    if (transient_qos) {
      m_qos.durability = QOSAbstraction::Durability::TRANSIENT_LOCAL;
    } else {
      m_qos.durability = QOSAbstraction::Durability::VOLATILE;
    }
    if (keep_last_qos) {
      m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_LAST;
    } else {
      m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_ALL;
    }
    m_qos.history_depth = history_depth;
    if (disable_async) {
      if (use_ros2_layers()) {
        throw std::invalid_argument("ROS 2 does not support disabling async. publishing.");
      }
      m_qos.sync_pubsub = true;
    }

    if (m_number_of_publishers > 1) {
      throw std::invalid_argument("More than one publisher is not supported at the moment");
    }

    if (m_expected_num_pubs > 1) {
      throw std::invalid_argument("More than one publisher is not supported at the moment");
    }

    if (prio != 0 || cpus != 0) {
#if PERFORMANCE_TEST_RT_ENABLED
      pre_proc_rt_init(cpus, prio);
      m_is_rt_init_required = true;
#else
      throw std::invalid_argument("Built with RT optimizations disabled");
#endif
    }
    if (m_use_single_participant) {
      if (use_ros2_layers()) {
        throw std::invalid_argument("ROS2 does not support single participant mode!");
      }
    }

    if (m_with_security) {
      if (!use_ros2_layers()) {
        throw std::invalid_argument("Only ROS2 supports security!");
      }
    }

    if (m_is_zero_copy_transfer) {
      if (m_number_of_publishers > 0 && m_number_of_subscribers > 0) {
        throw std::invalid_argument(
                "Zero copy transfer only makes sense for interprocess communication!");
      }
    }

    m_roundtrip_mode = RoundTripMode::NONE;
    const auto mode = roundtrip_mode_str;
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
#if defined DATABASE_MYSQL || defined DATABASE_PGSQL
    if (m_db_user.empty() || m_db_password.empty() || m_db_host.empty() ||
      m_db_port == 0)
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
    if (!m_logfile.empty()) {
      open_file();
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR: ";
    std::cerr << e.what() << std::endl;
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
bool ExperimentConfiguration::use_ros2_layers() const
{
#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  if (m_com_mean == CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR ||
    m_com_mean == CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR ||
    m_com_mean == CommunicationMean::RCLCPP_WAITSET)
  {
    return true;
  }
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
  if (com_mean() == CommunicationMean::ApexOSPollingSubscription) {
    return true;
  }
#endif
  return false;
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
std::string ExperimentConfiguration::msg_name() const
{
  check_setup();
  return m_msg_name;
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

bool ExperimentConfiguration::is_zero_copy_transfer() const
{
  check_setup();
  return m_is_zero_copy_transfer;
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
  return use_ros2_layers() && !rclcpp::ok();
}

}  // namespace performance_test
