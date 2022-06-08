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

#ifndef UTILITIES__JSON_LOGGER_HPP_
#define UTILITIES__JSON_LOGGER_HPP_

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <ostream>
#include <string>
#include <chrono>
#include <memory>
#include <vector>

#include "../experiment_configuration/experiment_configuration.hpp"
#include "../experiment_execution/analysis_result.hpp"

namespace performance_test
{

///  Calculate the CPU usage for the running experiment in the performance test
class JsonLogger
{
public:
  static void log(
    const ExperimentConfiguration & ec,
    const std::vector<std::shared_ptr<const AnalysisResult>> & ars,
    std::ostream & stream)
  {
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);

    writer.StartObject();

    write(writer, "id", ec.id());
    write(writer, "perf_test_version", ec.perf_test_version());
    write(writer, "final_logfile_name", tableau_final_logfile_name(ec.id(), ec.topic_name()));
    write(writer, "com_mean_str", to_string(ec.com_mean()));
    write(writer, "rmw_implementation", ec.rmw_implementation());
    write(writer, "dds_domain_id", ec.dds_domain_id());
    write(writer, "qos_reliability", to_string(ec.qos().reliability));
    write(writer, "qos_durability", to_string(ec.qos().durability));
    write(writer, "qos_history_kind", to_string(ec.qos().history_kind));
    write(writer, "qos_history_depth", ec.qos().history_depth);
    write(writer, "qos_sync_pubsub", ec.qos().sync_pubsub);
    write(writer, "rate", ec.rate());
    write(writer, "topic_name", ec.topic_name());
    write(writer, "msg_name", ec.msg_name());
    write(writer, "max_runtime", ec.max_runtime());
    write(writer, "number_of_publishers", ec.number_of_publishers());
    write(writer, "number_of_subscribers", ec.number_of_subscribers());
    write(writer, "check_memory", ec.check_memory());
    // TODO(erik.snider) delete when gc parse_upload is fixed
    write(writer, "use_single_participant", false);
    write(writer, "with_security", ec.is_with_security());
    write(writer, "is_zero_copy_transfer", ec.is_zero_copy_transfer());
    write(writer, "roundtrip_mode", to_string(ec.roundtrip_mode()));
    write(writer, "is_rt_init_required", ec.is_rt_init_required());
    write(writer, "external_info_githash", ec.get_external_info().m_githash);
    write(writer, "external_info_platform", ec.get_external_info().m_platform);
    write(writer, "external_info_branch", ec.get_external_info().m_branch);
    write(writer, "external_info_architecture", ec.get_external_info().m_architecture);
    write(writer, "external_info_ci", ec.get_external_info().m_ci);

    writer.String("analysis_results");
    writer.StartArray();
    for (const auto & ar : ars) {
      writer.StartObject();
      write(writer, "experiment_start", ar->m_experiment_start);
      write(writer, "loop_start", ar->m_time_between_two_measurements);
      write(writer, "num_samples_received", ar->m_num_samples_received);
      write(writer, "num_samples_sent", ar->m_num_samples_sent);
      write(writer, "num_samples_lost", ar->m_num_samples_lost);
      write(writer, "total_data_received", ar->m_total_data_received);
      write(writer, "latency_min", ar->m_latency.min());
      write(writer, "latency_max", ar->m_latency.max());
      write(writer, "latency_n", ar->m_latency.n());
      write(writer, "latency_mean", ar->m_latency.mean());
      write(writer, "latency_M2", ar->m_latency.m2());
      write(writer, "latency_variance", ar->m_latency.variance());
#if !defined(WIN32)
      write(writer, "sys_tracker_ru_utime", ar->m_sys_usage.ru_utime);
      write(writer, "sys_tracker_ru_stime", ar->m_sys_usage.ru_stime);
      write(writer, "sys_tracker_ru_maxrss", ar->m_sys_usage.ru_maxrss);
      write(writer, "sys_tracker_ru_ixrss", ar->m_sys_usage.ru_ixrss);
      write(writer, "sys_tracker_ru_idrss", ar->m_sys_usage.ru_idrss);
      write(writer, "sys_tracker_ru_isrss", ar->m_sys_usage.ru_isrss);
      write(writer, "sys_tracker_ru_minflt", ar->m_sys_usage.ru_minflt);
      write(writer, "sys_tracker_ru_majflt", ar->m_sys_usage.ru_majflt);
      write(writer, "sys_tracker_ru_nswap", ar->m_sys_usage.ru_nswap);
      write(writer, "sys_tracker_ru_inblock", ar->m_sys_usage.ru_inblock);
      write(writer, "sys_tracker_ru_oublock", ar->m_sys_usage.ru_oublock);
      write(writer, "sys_tracker_ru_msgsnd", ar->m_sys_usage.ru_msgsnd);
      write(writer, "sys_tracker_ru_msgrcv", ar->m_sys_usage.ru_msgrcv);
      write(writer, "sys_tracker_ru_nsignals", ar->m_sys_usage.ru_nsignals);
      write(writer, "sys_tracker_ru_nvcsw", ar->m_sys_usage.ru_nvcsw);
      write(writer, "sys_tracker_ru_nivcsw", ar->m_sys_usage.ru_nivcsw);
#endif
      write(writer, "cpu_info_cpu_cores", ar->m_cpu_info.cpu_cores());
      write(writer, "cpu_info_cpu_usage", ar->m_cpu_info.cpu_usage());
      writer.EndObject();
    }
    writer.EndArray();

    writer.EndObject();

    stream << sb.GetString();
  }

private:
  // Tableau parses the date and time out of the final_logfile_name column.
  // This workaround feel bad.
  // It would be much better if there were a dedicated datetime column.
  static std::string tableau_final_logfile_name(const std::string & id, const std::string & topic)
  {
    auto t = std::time(nullptr);
    auto tm = *std::gmtime(&t);
    std::ostringstream oss;
    oss << id << "_" << topic << std::put_time(&tm, "_%d-%m-%Y_%H-%M-%S");
    return oss.str();
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, const std::string & val)
  {
    writer.String(key);
    writer.String(val.c_str());
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, uint32_t val)
  {
    writer.String(key);
    writer.Uint(val);
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, uint64_t val)
  {
    writer.String(key);
    writer.Uint64(val);
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, int32_t val)
  {
    writer.String(key);
    writer.Int(val);
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, int64_t val)
  {
    writer.String(key);
    writer.Int64(val);
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, float val)
  {
    writer.String(key);
    if (std::isfinite(val)) {
      writer.Double(val);
    } else {
      writer.Double(0.0);
    }
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, double val)
  {
    writer.String(key);
    if (std::isfinite(val)) {
      writer.Double(val);
    } else {
      writer.Double(0.0);
    }
  }

  template<typename Writer>
  static void write(Writer & writer, const char * key, bool val)
  {
    writer.String(key);
    writer.Bool(val);
  }

#if !defined(WIN32)
  template<typename Writer>
  static void write(Writer & writer, const char * key, timeval val)
  {
    writer.String(key);
    std::chrono::nanoseconds ns =
      std::chrono::seconds(val.tv_sec) +
      std::chrono::microseconds(val.tv_usec);
    writer.Int64(ns.count());
  }
#endif

  template<typename Writer>
  static void write(Writer & writer, const char * key, const std::chrono::nanoseconds val)
  {
    writer.String(key);
    writer.Int64(val.count());
  }
};
}  // namespace performance_test

#endif  // UTILITIES__JSON_LOGGER_HPP_
