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

#include "resource_manager.hpp"

#if defined(PERFORMANCE_TEST_FASTRTPS_ENABLED) && !defined(USE_LEGACY_QOS_API)
  #include <fastrtps/rtps/attributes/RTPSParticipantAttributes.h>
#endif

#include <cstdlib>
#include <memory>
#include <string>

namespace performance_test
{

std::shared_ptr<rclcpp::Node> ResourceManager::ros2_node() const
{
  /* Temporarely commented out until ROS2 waitsets are available. As of now every ROS2 thread needs a node in the
   * current architecture.
   */

//    std::lock_guard<std::mutex> lock(m_global_mutex);
//    if(!m_node)
//    {
//      m_node = rclcpp::Node::make_shared("performance_test", "", m_ec.use_ros_shm());
//    }
//    return m_node;

  std::string rand_str;
  // if security is enabled
  if (m_ec.is_with_security()) {
    static uint32_t id = 0;
    rand_str = std::to_string(id++);
  } else {
    rand_str = std::to_string(std::rand());
  }

  auto options = rclcpp::NodeOptions().use_intra_process_comms(m_ec.use_ros_shm());

  auto env_name = "ROS_DOMAIN_ID";
  auto env_value = std::to_string(m_ec.dds_domain_id());
#ifdef _WIN32
  _putenv_s(env_name, env_value.c_str());
#else
  setenv(env_name, env_value.c_str(), true);
#endif

  return rclcpp::Node::make_shared("performance_test" + rand_str, options);
}

#ifdef PERFORMANCE_TEST_FASTRTPS_ENABLED
eprosima::fastrtps::Participant * ResourceManager::fastrtps_participant() const
{
  std::lock_guard<std::mutex> lock(m_global_mutex);

  eprosima::fastrtps::Participant * result = nullptr;
  eprosima::fastrtps::ParticipantAttributes PParam;

  eprosima::fastrtps::xmlparser::XMLProfileManager::loadDefaultXMLFile();
  eprosima::fastrtps::xmlparser::XMLProfileManager::getDefaultParticipantAttributes(PParam);
  PParam.rtps.sendSocketBufferSize = 1048576;
  PParam.rtps.listenSocketBufferSize = 4194304;
#ifdef USE_LEGACY_QOS_API
  PParam.rtps.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = true;
  PParam.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = true;
  PParam.rtps.builtin.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
  PParam.rtps.builtin.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
  PParam.rtps.builtin.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
#else
  eprosima::fastrtps::rtps::DiscoverySettings & disc_config = PParam.rtps.builtin.discovery_config;
  disc_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
  disc_config.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
  disc_config.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
  disc_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
#endif
#if FASTRTPS_VERSION_MAJOR < 2
  PParam.rtps.builtin.domainId = m_ec.dds_domain_id();
#else
  PParam.domainId = m_ec.dds_domain_id();
#endif
  PParam.rtps.setName("performance_test_fastRTPS");

  if (!m_ec.use_single_participant()) {
    result = eprosima::fastrtps::Domain::createParticipant(PParam);
  } else {
    if (!m_fastrtps_participant) {
      m_fastrtps_participant = eprosima::fastrtps::Domain::createParticipant(PParam);
    }
    result = m_fastrtps_participant;
  }
  return result;
}
#endif

#ifdef PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED
DDSDomainParticipant * ResourceManager::connext_DDS_micro_participant() const
{
  std::lock_guard<std::mutex> lock(m_global_mutex);

  if (!m_connext_dds_micro_participant) {
    m_connext_dds_micro_participant = &apex::ConnextMicroContext::get_domain_participant();
    if (m_connext_dds_micro_participant == nullptr) {
      throw std::runtime_error("failed to create participant");
    }
  }
  return m_connext_dds_micro_participant;
}

void ResourceManager::connext_dds_micro_publisher(
  DDSPublisher * & publisher,
  DDS_DataWriterQos & dw_qos) const
{
  auto participant = connext_DDS_micro_participant();
  std::lock_guard<std::mutex> lock(m_global_mutex);
  publisher = participant->create_publisher(
    DDS_PUBLISHER_QOS_DEFAULT, nullptr, DDS_STATUS_MASK_NONE);
  if (publisher == nullptr) {
    throw std::runtime_error("Pulisher is nullptr");
  }
  auto retcode = publisher->get_default_datawriter_qos(dw_qos);
  if (retcode != DDS_RETCODE_OK) {
    throw std::runtime_error("Failed to get default datawriter");
  }
}

void ResourceManager::connext_dds_micro_subscriber(
  DDSSubscriber * & subscriber,
  DDS_DataReaderQos & dr_qos) const
{
  auto participant = connext_DDS_micro_participant();
  std::lock_guard<std::mutex> lock(m_global_mutex);
  subscriber = participant->create_subscriber(
    DDS_SUBSCRIBER_QOS_DEFAULT, nullptr,
    DDS_STATUS_MASK_NONE);
  if (subscriber == nullptr) {
    throw std::runtime_error("m_subscriber == nullptr");
  }
  auto retcode = subscriber->get_default_datareader_qos(dr_qos);
  if (retcode != DDS_RETCODE_OK) {
    throw std::runtime_error("failed get_default_datareader_qos");
  }
}
#endif

#ifdef PERFORMANCE_TEST_CYCLONEDDS_ENABLED
dds_entity_t ResourceManager::cyclonedds_participant() const
{
  std::lock_guard<std::mutex> lock(m_global_mutex);

  dds_entity_t result = 0;

  if (!m_ec.use_single_participant()) {
    result = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
  } else {
    if (!m_cyclonedds_participant) {
      result = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
    }
    result = m_cyclonedds_participant;
  }
  return result;
}
#endif

#ifdef PERFORMANCE_TEST_OPENDDS_ENABLED
DDS::DomainParticipant_ptr
ResourceManager::opendds_participant() const
{
  std::lock_guard<std::mutex> lock(m_global_mutex);

  if (CORBA::is_nil(m_opendds_participant)) {
    DDS::DomainParticipantFactory_var dpf = TheParticipantFactory;

    OpenDDS::DCPS::TransportConfig_rch config =
      OpenDDS::DCPS::TransportRegistry::instance()->create_config("ApexAiConfig");
    OpenDDS::DCPS::TransportInst_rch inst =
      OpenDDS::DCPS::TransportRegistry::instance()->create_inst("rtps_tran", "rtps_udp");
    OpenDDS::DCPS::RtpsUdpInst_rch rui =
      OpenDDS::DCPS::static_rchandle_cast<OpenDDS::DCPS::RtpsUdpInst>(inst);
    rui->handshake_timeout_ = 1;

    config->instances_.push_back(inst);
    OpenDDS::DCPS::TransportRegistry::instance()->global_config(config);

    int domain = m_ec.dds_domain_id();
    bool multicast = true;
    unsigned int resend = 1;
    std::string partition, governance, permissions;
    int defaultSize = 0;

    OpenDDS::RTPS::RtpsDiscovery_rch disc;
    disc = OpenDDS::DCPS::make_rch<OpenDDS::RTPS::RtpsDiscovery>("RtpsDiscovery");
    rui->use_multicast_ = true;
    rui->local_address("127.0.0.1:");
    rui->multicast_interface_ = "lo";
    disc->sedp_multicast(true);

    TheServiceParticipant->add_discovery(
      OpenDDS::DCPS::static_rchandle_cast<OpenDDS::DCPS::Discovery>(disc));
    TheServiceParticipant->set_repo_domain(domain, disc->key());
    DDS::DomainParticipantQos dp_qos;
    dpf->get_default_participant_qos(dp_qos);
    m_opendds_participant = dpf->create_participant(
      m_ec.dds_domain_id(),
      PARTICIPANT_QOS_DEFAULT,
      nullptr,
      OpenDDS::DCPS::DEFAULT_STATUS_MASK);
  }
  return m_opendds_participant;
}

void
ResourceManager::opendds_publisher(
  DDS::Publisher_ptr & publisher,
  DDS::DataWriterQos & dw_qos) const
{
  DDS::DomainParticipant_ptr participant = opendds_participant();
  std::lock_guard<std::mutex> lock(m_global_mutex);

  publisher = participant->create_publisher(
    PUBLISHER_QOS_DEFAULT,
    nullptr,
    OpenDDS::DCPS::DEFAULT_STATUS_MASK);

  if (CORBA::is_nil(publisher)) {
    throw std::runtime_error("Failed to create publisher");
  }

  DDS::ReturnCode_t ret;
  ret = publisher->get_default_datawriter_qos(dw_qos);
  if (ret != DDS::RETCODE_OK) {
    throw std::runtime_error("Failed to get default datawriter qos");
  }
}

void
ResourceManager::opendds_subscriber(
  DDS::Subscriber_ptr & subscriber,
  DDS::DataReaderQos & dr_qos) const
{
  DDS::DomainParticipant_ptr participant = opendds_participant();
  std::lock_guard<std::mutex> lock(m_global_mutex);

  subscriber = participant->create_subscriber(
    SUBSCRIBER_QOS_DEFAULT,
    nullptr,
    OpenDDS::DCPS::DEFAULT_STATUS_MASK);

  if (CORBA::is_nil(subscriber)) {
    throw std::runtime_error("Failed to create subscriber");
  }

  DDS::ReturnCode_t ret;
  ret = subscriber->get_default_datareader_qos(dr_qos);
  if (ret != DDS::RETCODE_OK) {
    throw std::runtime_error("Failed to get default datareader qos");
  }
}
#endif
}  // namespace performance_test
