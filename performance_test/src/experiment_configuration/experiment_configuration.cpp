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

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
#include <rmw/rmw.h>
#endif

#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
#include <settings/inspect.hpp>
#include <settings/repository.hpp>
#include <cyclone_dds_vendor/dds.hpp>
#endif

#include <iostream>
#include <iomanip>
#include <exception>
#include <regex>
#include <string>
#include <vector>
#include <memory>
#include <sole/sole.hpp>

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
#include <rclcpp/rclcpp.hpp>
#endif

#include <performance_test/generated_messages/messages.hpp>

#include "../outputs/csv_output.hpp"
#include "../outputs/stdout_output.hpp"
#include "../outputs/json_output.hpp"

#include "performance_test/version.h"

namespace performance_test
{

std::string to_string(const ExperimentConfiguration::RoundTripMode e)
{
  if (e == ExperimentConfiguration::RoundTripMode::MAIN) {
    return "MAIN";
  } else if (e == ExperimentConfiguration::RoundTripMode::RELAY) {
    return "RELAY";
  } else {
    return "NONE";
  }
}

std::ostream & operator<<(std::ostream & stream, const ExperimentConfiguration::RoundTripMode & e)
{
  return stream << to_string(e);
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
           "\nWith security: " << e.is_with_security() <<
           "\nShared memory transfer: " << e.is_shared_memory_transfer() <<
           "\nZero copy transfer: " << e.is_zero_copy_transfer() <<
           "\nUnbounded message size: " << e.unbounded_msg_size() <<
           "\nRoundtrip Mode: " << e.roundtrip_mode() <<
           "\nIgnore seconds from beginning: " << e.rows_to_ignore();
  } else {
    return stream << "ERROR: Experiment is not yet setup!";
  }
}

ExperimentConfiguration::ExperimentConfiguration()
: m_id(sole::uuid4().str()),
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
  m_is_rt_init_required(false),
  m_is_zero_copy_transfer(false),
  m_roundtrip_mode(RoundTripMode::NONE)
{}

void ExperimentConfiguration::setup(int argc, char ** argv)
{
  std::string comm_str;
  bool print_msg_list = false;
  bool reliable_qos = false;
  std::string reliability_qos;
  std::string durability_qos;
  std::string history_qos;
  bool transient_qos = false;
  bool keep_last_qos = false;
  uint32_t history_depth = 0;
  bool disable_async = false;
  int32_t prio = 0;
  uint32_t cpus = 0;
  std::string roundtrip_mode_str;
  try {
    TCLAP::CmdLine cmd("Apex.AI performance_test");

    TCLAP::SwitchArg printToConsoleArg("", "print-to-console",
      "Print metrics to console.", cmd, false);

    TCLAP::ValueArg<std::string> LogfileArg("l", "logfile",
      "Specify the name of the log file, e.g. -l \"log_$(date +%F_%H-%M-%S).json\"."
      " Supported formats: csv, json", false, "", "name", cmd);

    TCLAP::ValueArg<uint32_t> rateArg("r", "rate",
      "The publishing rate. 0 means publish as fast as possible. "
      "Default is 1000.", false, 1000, "N", cmd);

    std::vector<std::string> allowedCommunications;

#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
    allowedCommunications.push_back("rclcpp-single-threaded-executor");
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
    allowedCommunications.push_back("rclcpp-static-single-threaded-executor");
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
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
#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
    allowedCommunications.push_back("CycloneDDS-CXX");
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
    allowedCommunications.push_back("iceoryx");
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
    allowedCommunications.push_back("OpenDDS");
#endif
    TCLAP::ValuesConstraint<std::string> allowedCommunicationVals(allowedCommunications);
    TCLAP::ValueArg<std::string> communicationArg("c", "communication",
      "The communication plugin to use. "
      "Default is " + allowedCommunications[0] + ".", false, allowedCommunications[0],
      &allowedCommunicationVals, cmd);

    TCLAP::ValueArg<std::string> topicArg("t", "topic", "The topic name. Default is test_topic.",
      false, "test_topic", "topic", cmd);

    TCLAP::ValueArg<std::string> msgArg("m", "msg",
      "The message type. Use --msg-list to list the options. "
      "Default is Array1k.", false, "Array1k", "type", cmd);

    TCLAP::SwitchArg msgListArg("", "msg-list",
      "Print the list of available msg types and exit.", cmd, false);

    TCLAP::ValueArg<uint32_t> ddsDomainIdArg("", "dds-domain_id",
      "The DDS domain id. Default is 0.", false, 0, "id", cmd);

    TCLAP::SwitchArg reliableArg("", "reliable",
      "DEPRECATED. Please use --reliability RELIABLE instead.", cmd, false);

    TCLAP::SwitchArg transientArg("", "transient",
      "DEPRECATED. Please use --durability TRANSIENT_LOCAL instead.", cmd, false);

    TCLAP::SwitchArg keepLastArg("", "keep-last",
      "DEPRECATED. Please use --history KEEP_LAST instead.", cmd, false);

    std::vector<std::string> allowedReliabilityArgs{"RELIABLE", "BEST_EFFORT"};
    TCLAP::ValuesConstraint<std::string> allowedReliabilityArgsVals(allowedReliabilityArgs);
    TCLAP::ValueArg<std::string> reliabilityArg("", "reliability",
      "The QOS Reliability type. Default is BEST_EFFORT.", false, "BEST_EFFORT",
      &allowedReliabilityArgsVals, cmd);

    std::vector<std::string> allowedDurabilityArgs{"TRANSIENT_LOCAL", "VOLATILE"};
    TCLAP::ValuesConstraint<std::string> allowedDurabilityArgsVals(allowedDurabilityArgs);
    TCLAP::ValueArg<std::string> durabilityArg("", "durability",
      "The QOS Durability type. Default is VOLATILE.", false, "VOLATILE",
      &allowedDurabilityArgsVals, cmd);

    std::vector<std::string> allowedHistoryArgs{"KEEP_LAST", "KEEP_ALL"};
    TCLAP::ValuesConstraint<std::string> allowedHistoryArgsVals(allowedHistoryArgs);
    TCLAP::ValueArg<std::string> historyArg("", "history",
      "The QOS History type. Default is KEEP_ALL.", false, "KEEP_ALL",
      &allowedHistoryArgsVals, cmd);

    TCLAP::ValueArg<uint32_t> historyDepthArg("", "history-depth",
      "The history depth QOS. Default is 1000.", false, 1000, "N", cmd);

    TCLAP::SwitchArg disableAsyncArg("", "disable-async",
      "Disable asynchronous pub/sub.", cmd, false);

    TCLAP::ValueArg<uint64_t> maxRuntimeArg("", "max-runtime",
      "Run N seconds, then exit. 0 means run forever. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> numPubsArg("p", "num-pub-threads",
      "Number of publisher threads. Default is 1.", false, 1, "N", cmd);

    TCLAP::ValueArg<uint32_t> numSubsArg("s", "num-sub-threads",
      "Number of subscriber threads. Default is 1.", false, 1, "N", cmd);

    TCLAP::SwitchArg checkMemoryArg("", "check-memory",
      "Print backtrace of all memory operations performed by the middleware. "
      "This will slow down the application!", cmd, false);

    TCLAP::ValueArg<int32_t> useRtPrioArg("", "use-rt-prio",
      "Set RT priority. "
      "Only certain platforms (i.e. Drive PX) have the right "
      "configuration to support this. Default is 0 (disabled).",
      false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> useRtCpusArg("", "use-rt-cpus",
      "Set RT CPU affinity mask. "
      "Only certain platforms (i.e. Drive PX) have the right "
      "configuration to support this. Default is 0 (disabled).",
      false, 0, "N", cmd);

    TCLAP::SwitchArg withSecurityArg("", "with-security",
      "Make nodes with deterministic names for use with security.", cmd, false);

    std::vector<std::string> allowedRelayModes{{"None", "Main", "Relay"}};
    TCLAP::ValuesConstraint<std::string> allowedRelayModeVals(allowedRelayModes);
    TCLAP::ValueArg<std::string> relayModeArg("", "roundtrip-mode",
      "Select the round trip mode. Default is None.", false, "None",
      &allowedRelayModeVals, cmd);

    TCLAP::ValueArg<uint32_t> ignoreArg("", "ignore",
      "Ignore the first N seconds of the experiment. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> expectedNumPubsArg("", "expected-num-pubs",
      "Expected number of publishers for wait-for-matched. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> expectedNumSubsArg("", "expected-num-subs",
      "Expected number of subscribers for wait-for-matched. Default is 0.", false, 0, "N", cmd);

    TCLAP::ValueArg<uint32_t> waitForMatchedTimeoutArg("", "wait-for-matched-timeout",
      "Maximum time in seconds to wait for matched pubs/subs. Default is 30.", false, 30, "N", cmd);

    TCLAP::SwitchArg zeroCopyArg("", "zero-copy",
      "Use zero copy transfer.", cmd, false);

    TCLAP::ValueArg<uint32_t> unboundedMsgSizeArg("", "unbounded-msg-size",
      "The number of bytes to use for an unbounded message type. "
      "Ignored for other messages. Default is 0.",
      false, 0, "N", cmd);

    cmd.parse(argc, argv);

    m_rate = rateArg.getValue();
    comm_str = communicationArg.getValue();
    m_logfile_name = LogfileArg.getValue();
    m_topic_name = topicArg.getValue();
    m_msg_name = msgArg.getValue();
    print_msg_list = msgListArg.getValue();
    m_dds_domain_id = ddsDomainIdArg.getValue();
    reliable_qos = reliableArg.getValue();
    reliability_qos = reliabilityArg.getValue();
    durability_qos = durabilityArg.getValue();
    history_qos = historyArg.getValue();
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
    m_with_security = withSecurityArg.getValue();
    roundtrip_mode_str = relayModeArg.getValue();
    m_rows_to_ignore = ignoreArg.getValue();
    m_expected_num_pubs = expectedNumPubsArg.getValue();
    m_expected_num_subs = expectedNumSubsArg.getValue();
    m_wait_for_matched_timeout = waitForMatchedTimeoutArg.getValue();
    m_is_zero_copy_transfer = zeroCopyArg.getValue();
    m_unbounded_msg_size = unboundedMsgSizeArg.getValue();

    // Configure outputs
    if (printToConsoleArg.getValue()) {
      std::cout << "WARNING: Printing to the console degrades the performance." << std::endl;
      std::cout << "It is recommended to use a log file instead with --logfile." << std::endl;
      m_configured_outputs.push_back(std::make_shared<StdoutOutput>());
    }

    if (!m_logfile_name.empty()) {
      if (std::regex_match(m_logfile_name, std::regex(".*\\.csv$"))) {
        m_configured_outputs.push_back(std::make_shared<CsvOutput>());
      } else if (std::regex_match(m_logfile_name, std::regex(".*\\.json$"))) {
        m_configured_outputs.push_back(std::make_shared<JsonOutput>());
      } else {
        std::cerr << "Unsupported log file type: " << m_logfile_name << std::endl;
        std::terminate();
      }
    }

    if (m_configured_outputs.empty()) {
      std::cout << "WARNING: No output configured" << std::endl;
    }
  } catch (TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  m_perf_test_version = version;

  try {
    if (print_msg_list) {
      for (const auto & s : messages::supported_msg_names()) {
        std::cout << s << std::endl;
      }
      // Exiting as we just print out some information and not running the
      // application.
      exit(0);
    }

#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
    if (comm_str == "rclcpp-single-threaded-executor") {
      m_com_mean = CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR;
    }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
    if (comm_str == "rclcpp-static-single-threaded-executor") {
      m_com_mean = CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR;
    }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
    if (comm_str == "rclcpp-waitset") {
      m_com_mean = CommunicationMean::RCLCPP_WAITSET;
    }
#endif
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    if (comm_str == "ApexOSPollingSubscription") {
      m_com_mean = CommunicationMean::ApexOSPollingSubscription;
    }
#endif
#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
    if (comm_str == "FastRTPS") {
      m_com_mean = CommunicationMean::FASTRTPS;
    }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
    if (comm_str == "ConnextDDSMicro") {
      m_com_mean = CommunicationMean::CONNEXTDDSMICRO;
    }
#endif
#ifdef PERFORMANCE_TEST_CONNEXTDDS_ENABLED
    if (comm_str == "ConnextDDS") {
      m_com_mean = CommunicationMean::CONNEXTDDS;
    }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
    if (comm_str == "CycloneDDS") {
      m_com_mean = CommunicationMean::CYCLONEDDS;
    }
#endif
#ifdef PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED
    if (comm_str == "CycloneDDS-CXX") {
      m_com_mean = CommunicationMean::CYCLONEDDS_CXX;
    }
#endif
#ifdef PERFORMANCE_TEST_ICEORYX_ENABLED
    if (comm_str == "iceoryx") {
      m_com_mean = CommunicationMean::ICEORYX;
    }
#endif
#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
    if (comm_str == "OpenDDS") {
      m_com_mean = CommunicationMean::OPENDDS;
    }
#endif

    if (reliable_qos) {
      m_qos.reliability = QOSAbstraction::Reliability::RELIABLE;
      std::cout << "WARNING: The flag '--reliable' is deprecated.\n"
        "Please use '--reliability RELIABLE' instead.\n";
    } else {
      if (reliability_qos == "RELIABLE") {
        m_qos.reliability = QOSAbstraction::Reliability::RELIABLE;
      } else if (reliability_qos == "BEST_EFFORT") {
        m_qos.reliability = QOSAbstraction::Reliability::BEST_EFFORT;
      }
    }

    if (transient_qos) {
      m_qos.durability = QOSAbstraction::Durability::TRANSIENT_LOCAL;
      std::cout << "WARNING: The flag '--transient' is deprecated.\n"
        "Please use '--durability TRANSIENT_LOCAL' instead.\n";
    } else {
      if (durability_qos == "VOLATILE") {
        m_qos.durability = QOSAbstraction::Durability::VOLATILE;
      } else if (durability_qos == "TRANSIENT_LOCAL") {
        m_qos.durability = QOSAbstraction::Durability::TRANSIENT_LOCAL;
      }
    }

    if (keep_last_qos) {
      m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_LAST;
      std::cout << "WARNING: The flag '--keep-last' is deprecated.\n"
        "Please use '--history KEEP_LAST' instead.\n";
    } else {
      if (history_qos == "KEEP_LAST") {
        m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_LAST;
      } else if (history_qos == "KEEP_ALL") {
        m_qos.history_kind = QOSAbstraction::HistoryKind::KEEP_ALL;
      }
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

    if (m_with_security) {
      if (!use_ros2_layers()) {
        throw std::invalid_argument("Only ROS2 supports security!");
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

#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
    m_rmw_implementation = rmw_get_implementation_identifier();
#else
    m_rmw_implementation = "N/A";
#endif
    m_is_setup = true;
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
#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
  if (m_com_mean == CommunicationMean::RCLCPP_SINGLE_THREADED_EXECUTOR) {
    return true;
  }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
  if (m_com_mean == CommunicationMean::RCLCPP_STATIC_SINGLE_THREADED_EXECUTOR) {
    return true;
  }
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED
  if (m_com_mean == CommunicationMean::RCLCPP_WAITSET) {
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

bool ExperimentConfiguration::is_shared_memory_transfer() const
{
  check_setup();
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
#ifdef DDSCXX_HAS_SHM
  if (rmw_implementation() == "rmw_apex_middleware" && use_ros2_layers()) {
    return apex::settings::inspect::get_or_default<bool>(
      apex::settings::repository::get(), "domain/shared_memory/enable", false);
  }
#endif
#endif
  return false;
}

bool ExperimentConfiguration::is_zero_copy_transfer() const
{
  check_setup();
  return m_is_zero_copy_transfer;
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

std::string ExperimentConfiguration::id() const
{
  return m_id;
}

std::string ExperimentConfiguration::logfile_name() const
{
  check_setup();
  return m_logfile_name;
}

const std::vector<std::shared_ptr<Output>> &
ExperimentConfiguration::configured_outputs() const
{
  check_setup();
  return m_configured_outputs;
}

size_t ExperimentConfiguration::unbounded_msg_size() const
{
  check_setup();
  return m_unbounded_msg_size;
}

void ExperimentConfiguration::check_setup() const
{
  if (!m_is_setup) {
    throw std::runtime_error("Experiment is not yet setup!");
  }
}

bool ExperimentConfiguration::exit_requested() const
{
#ifdef PERFORMANCE_TEST_RCLCPP_ENABLED
  return use_ros2_layers() && !rclcpp::ok();
#else
  return false;
#endif
}

}  // namespace performance_test
