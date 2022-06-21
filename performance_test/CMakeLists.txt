
# Copyright 2017 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(performance_test)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
include(compile_options)

set(OPTIONAL_AMENT_DEPENDENCES)
set(OPTIONAL_LIBRARIES)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
find_package(ament_cmake REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(PERFORMANCE_TEST_RT_ENABLED_DEFAULT ON)
  add_definitions(-DPERFORMANCE_TEST_LINUX)
else()
  set(PERFORMANCE_TEST_RT_ENABLED_DEFAULT OFF)
endif()
option(PERFORMANCE_TEST_RT_ENABLED
  "Enable options for thread and memory optimization. This may not build on all platforms"
  ${PERFORMANCE_TEST_RT_ENABLED_DEFAULT})

# Ensure we use the CMake support from ConnextDDS instead of the one provided
# with the installed RMW package.
# This change in the environment only affect to the CMake environment. The
# environment variable will remain as it is outside the building process.
if($ENV{RMW_IMPLEMENTATION})
  string(REPLACE
    "rmw_connext_cpp;"
    ""
    ENV{RMW_IMPLEMENTATION}
    $ENV{RMW_IMPLEMENTATION}
  )
  string(REPLACE
    "rmw_connext_cpp" # It can be alone, without other values
    ""
    ENV{RMW_IMPLEMENTATION}
    $ENV{RMW_IMPLEMENTATION}
  )
endif()

find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)

find_package(osrf_testing_tools_cpp QUIET)
if(${osrf_testing_tools_cpp_FOUND})
    list(APPEND OPTIONAL_AMENT_DEPENDENCES "osrf_testing_tools_cpp")
    list(APPEND OPTIONAL_LIBRARIES osrf_testing_tools_cpp::memory_tools)
    add_definitions(-DPERFORMANCE_TEST_MEMORYTOOLS_ENABLED)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

# This is a workaround for broken include paths on some systems.
include_directories(${FastRTPS_INCLUDE_DIR} ${FastRTPS_INCLUDE_DIR}/fastrtps/include ${fastcdr_INCLUDE_DIR})
include_directories(include ${osrf_testing_tools_cpp_INCLUDE_DIR})


# ROS2 rclcpp plugins
option(PERFORMANCE_TEST_RCLCPP_ENABLED "" ON)
if(PERFORMANCE_TEST_RCLCPP_ENABLED)
  find_package(rclcpp REQUIRED)
  find_package(rmw REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  add_definitions(-DPERFORMANCE_TEST_RCLCPP_ENABLED)
  set(PERFORMANCE_TEST_RCLCPP_STE_ENABLED ON)
  set(PERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED ON)
  if($ENV{ROS_DISTRO} MATCHES "dashing")
    set(PERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED OFF)
  endif()
  if($ENV{ROS_DISTRO} MATCHES "foxy")
    set(PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED ON)
  endif()
  if($ENV{ROS_DISTRO} MATCHES "galactic")
    set(PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED ON)
    set(PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED ON)
  endif()
  if($ENV{ROS_DISTRO} MATCHES "rolling")
    set(PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED ON)
    set(PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED ON)
  endif()
  if($ENV{ROS_DISTRO} MATCHES "humble")
    set(PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED ON)
    set(PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED ON)
  endif()
  if(PERFORMANCE_TEST_RCLCPP_STE_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_RCLCPP_STE_ENABLED)
  endif()
  if(PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_RCLCPP_SSTE_ENABLED)
  endif()
  if(PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED)
  endif()
  if(PERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED)
  endif()
endif()

# Apex.OS Waitset and Polling Subscription
option(PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED "Option to enable Polling Subscription Plugin.
  The plugin can only work if ApexOS is present, otherwise it will fail." OFF)

if(PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED)
  find_library(APEXCPP_LIB apexcpp)
  if(NOT APEXCPP_LIB)
    message(WARNING "ApexOSPollingSubscription not found, disabling")
    set(PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED OFF)
  else()
    find_package(rclcpp REQUIRED)
    find_package(rmw REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    add_definitions(-DPERFORMANCE_TEST_RCLCPP_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED)
  endif()
endif()

option(APEX_CERT "Set to true if building against Apex.OS Cert" OFF)
if(APEX_CERT)
  add_definitions(-DAPEX_CERT)
endif()

# FastRTPS
option(PERFORMANCE_TEST_FASTRTPS_ENABLED "Enable FastRTPS" OFF)
if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  find_package(fastrtps_cmake_module QUIET REQUIRED)
  find_package(FastRTPS QUIET REQUIRED)
  add_definitions(-DPERFORMANCE_TEST_FASTRTPS_ENABLED)
  list(APPEND PLUGIN_LIBRARIES ${FastRTPS_LIBRARIES})
endif()


# ConnextDDS
option(PERFORMANCE_TEST_CONNEXTDDS_ENABLED "Enable Connext DDS Pro" OFF)
if(PERFORMANCE_TEST_CONNEXTDDS_ENABLED)
    # Append the path to CMAKE_MODULE_PATH in order to find the
    # FindConnextDDS.cmake CMake script
    set(CMAKE_MODULE_PATH
      ${CMAKE_MODULE_PATH}
      "$ENV{NDDSHOME}/resource/cmake"
    )

    find_package(RTIConnextDDS
      REQUIRED
      COMPONENTS
        core
    )
    add_definitions(-DPERFORMANCE_TEST_CONNEXTDDS_ENABLED)
endif()


# RTI Connext Micro
option(PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED "Enable Connext DDS Micro" OFF)
if(PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
  find_package(connext_micro_cmake_module REQUIRED)
  find_package(ConnextMicro MODULE REQUIRED)
  if(ConnextMicro_FOUND)
    add_definitions(-DPERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
    ament_export_definitions(${ConnextMicro_DEFINITIONS})
  endif()
endif()


if(PERFORMANCE_TEST_CONNEXTDDS_ENABLED AND PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
  message(FATAL_ERROR
    "ConnextDDS Pro and ConnextDDS Micro cannot be used in the same "
    "application. Please, disable one")
endif()


# CycloneDDS
option(PERFORMANCE_TEST_CYCLONEDDS_ENABLED "Enable CycloneDDS" OFF)
if(PERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  find_package(CycloneDDS REQUIRED COMPONENTS idlc)
  add_definitions(-DPERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  list(APPEND PLUGIN_LIBRARIES CycloneDDS::ddsc)
endif()

# cyclonedds-cxx
option(PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED "Enable C++ binding for Cyclone DDS" OFF)
if(PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED)
  find_package(CycloneDDS-CXX REQUIRED)
  add_definitions(-DPERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED)
  list(APPEND PLUGIN_LIBRARIES cyclonedds_cxx_idl CycloneDDS-CXX::ddscxx)
endif()

# iceoryx
option(PERFORMANCE_TEST_ICEORYX_ENABLED "Require iceoryx to be available, fail otherwise" OFF)
if(PERFORMANCE_TEST_ICEORYX_ENABLED)
  find_package(iceoryx_posh CONFIG REQUIRED)
  add_definitions(-DPERFORMANCE_TEST_ICEORYX_ENABLED)
  list(APPEND PLUGIN_LIBRARIES iceoryx_posh::iceoryx_posh)
endif()


# OpenDDS
option(PERFORMANCE_TEST_OPENDDS_ENABLED "Require Open DDS to be available, fail otherwise" OFF)
if(PERFORMANCE_TEST_OPENDDS_ENABLED)
  add_definitions(-DPERFORMANCE_TEST_OPENDDS_ENABLED)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()


add_subdirectory(msg)
include_directories(${IDLGEN_INCLUDE_DIR})

if(PERFORMANCE_TEST_RCLCPP_ENABLED OR PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED)
  # rosidl_generate_interfaces must be invoked from this CMakeLists.txt file. Otherwise, the
  # generated messages are placed in a directory where ros1_bridge can not find them.
  rosidl_generate_interfaces(${PROJECT_NAME} ${ROSIDL_GEN_LIST})
endif()


set(sources
    src/main.cpp
    src/communication_abstractions/communicator.cpp
    src/communication_abstractions/communicator.hpp
    src/communication_abstractions/resource_manager.cpp
    src/communication_abstractions/resource_manager.hpp
    src/outputs/stdout_output.cpp
    src/outputs/csv_output.cpp
    src/outputs/json_output.cpp
    src/data_running/data_publisher.hpp
    src/data_running/data_subscriber.hpp
    src/data_running/data_entity.hpp
    src/data_running/data_entity_factory.cpp
    src/data_running/data_entity_factory.hpp
    src/experiment_execution/analysis_result.cpp
    src/experiment_execution/analysis_result.hpp
    src/experiment_execution/runner.hpp
    src/experiment_execution/data_stats.hpp
    src/experiment_execution/run_type.hpp
    src/experiment_configuration/communication_mean.cpp
    src/experiment_configuration/communication_mean.hpp
    src/experiment_configuration/qos_abstraction.cpp
    src/experiment_configuration/qos_abstraction.hpp
    src/experiment_configuration/experiment_configuration.cpp
    src/experiment_configuration/experiment_configuration.hpp
    src/experiment_configuration/external_info_storage.cpp
    src/experiment_configuration/external_info_storage.hpp
    src/utilities/spin_lock.hpp
    src/utilities/statistics_tracker.hpp
    src/utilities/cpu_usage_tracker.hpp
    src/utilities/qnx_res_usage.hpp
    src/utilities/json_logger.hpp
)

if(PERFORMANCE_TEST_RT_ENABLED)
  add_definitions(-DPERFORMANCE_TEST_RT_ENABLED)
  list(APPEND sources src/utilities/rt_enabler.hpp)
endif()

if(PERFORMANCE_TEST_RCLCPP_ENABLED)
  list(APPEND sources src/communication_abstractions/rclcpp_communicator.hpp)
  list(APPEND sources src/communication_abstractions/rclcpp_callback_communicator.hpp)
endif()

if(PERFORMANCE_TEST_RCLCPP_STE_ENABLED)
  list(APPEND sources src/data_running/factories/rclcpp_ste_data_runner_factory.cpp)
  list(APPEND sources src/data_running/factories/rclcpp_ste_data_runner_factory.hpp)
endif()

if(PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED)
  list(APPEND sources src/data_running/factories/rclcpp_sste_data_runner_factory.cpp)
  list(APPEND sources src/data_running/factories/rclcpp_sste_data_runner_factory.hpp)
endif()

if(PERFORMANCE_TEST_RCLCPP_WAITSET_ENABLED)
  list(APPEND sources src/communication_abstractions/rclcpp_waitset_communicator.hpp)
  list(APPEND sources src/data_running/factories/rclcpp_waitset_data_runner_factory.cpp)
  list(APPEND sources src/data_running/factories/rclcpp_waitset_data_runner_factory.hpp)
endif()

if(PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED)
  list(APPEND sources src/communication_abstractions/apex_os_polling_subscription_communicator.hpp)
endif()

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  list(APPEND sources src/communication_abstractions/fast_rtps_communicator.hpp)
endif()

if(PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
  list(APPEND sources src/communication_abstractions/connext_dds_micro_communicator.hpp)
endif()

if(PERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  list(APPEND sources src/communication_abstractions/cyclonedds_communicator.hpp)
endif()

if(PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED)
  list(APPEND sources src/communication_abstractions/cyclonedds_cxx_communicator.hpp)
endif()

if(PERFORMANCE_TEST_ICEORYX_ENABLED)
  list(APPEND sources src/communication_abstractions/iceoryx_communicator.hpp)
endif()

if(PERFORMANCE_TEST_OPENDDS_ENABLED)
  list(APPEND sources src/communication_abstractions/opendds_communicator.hpp)
endif()

include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
externalproject_add(
  tclap
  GIT_REPOSITORY http://git.code.sf.net/p/tclap/code
  GIT_TAG 1.4.0-rc1
  GIT_SHALLOW TRUE
  PREFIX ${EXTERNAL_INSTALL_LOCATION}
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/tclap
  CMAKE_COMMAND ""
  CONFIGURE_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_COMMAND ""
)
include_directories(SYSTEM ${EXTERNAL_INSTALL_LOCATION}/tclap/include)

externalproject_add(
  rapidjson
  GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
  GIT_TAG v1.1.0
  GIT_SHALLOW TRUE
  PREFIX ${EXTERNAL_INSTALL_LOCATION}
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/rapidjson
  CMAKE_COMMAND ""
  CMAKE_ARGS
    -DRAPIDJSON_BUILD_TESTS=OFF
    -DRAPIDJSON_BUILD_DOC=OFF
    -DRAPIDJSON_BUILD_EXAMPLES=OFF
  CONFIGURE_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_COMMAND ""
)
include_directories(SYSTEM ${EXTERNAL_INSTALL_LOCATION}/rapidjson/include)

externalproject_add(
  tabulate
  GIT_REPOSITORY https://github.com/p-ranav/tabulate.git
  GIT_TAG v1.4
  GIT_SHALLOW TRUE
  PREFIX ${EXTERNAL_INSTALL_LOCATION}
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/tabulate
  PATCH_COMMAND sed -i "39d" "${EXTERNAL_INSTALL_LOCATION}/tabulate/include/tabulate/column.hpp"
        COMMAND sed -i "88 a #endif" "${EXTERNAL_INSTALL_LOCATION}/tabulate/include/tabulate/utf8.hpp"
        COMMAND sed -i "77 a #if !defined(QNX)" "${EXTERNAL_INSTALL_LOCATION}/tabulate/include/tabulate/utf8.hpp"
        COMMAND sed -i "75 a #endif" "${EXTERNAL_INSTALL_LOCATION}/tabulate/include/tabulate/utf8.hpp"
        COMMAND sed -i "74 a #if !defined(QNX)" "${EXTERNAL_INSTALL_LOCATION}/tabulate/include/tabulate/utf8.hpp"
        COMMAND sed -i "71 a #endif" "${EXTERNAL_INSTALL_LOCATION}/tabulate/include/tabulate/utf8.hpp"
        COMMAND sed -i "47 a #if !defined(QNX)" "${EXTERNAL_INSTALL_LOCATION}/tabulate/include/tabulate/utf8.hpp"
  CMAKE_COMMAND ""
  CONFIGURE_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_COMMAND ""
)
include_directories(SYSTEM ${EXTERNAL_INSTALL_LOCATION}/tabulate/include)

externalproject_add(
  sole
  GIT_REPOSITORY https://github.com/r-lyeh-archived/sole.git
  GIT_TAG 1.0.2
  GIT_SHALLOW TRUE
  PREFIX ${EXTERNAL_INSTALL_LOCATION}
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/sole
  PATCH_COMMAND patch
        ${EXTERNAL_INSTALL_LOCATION}/sole/sole.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external_patches/sole.hpp.patch
  CMAKE_COMMAND ""
  CONFIGURE_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_COMMAND ""
)
include_directories(SYSTEM ${EXTERNAL_INSTALL_LOCATION})

set(EXE_NAME perf_test)
add_executable(${EXE_NAME} ${sources})
add_dependencies(${EXE_NAME} tclap rapidjson tabulate sole)


set_compile_options(${EXE_NAME})


# Try to bake the githash into the perf_test EXE:
execute_process(
  COMMAND ./version_check.bash
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE PERF_TEST_VERSION
)
add_definitions(-DPERFORMANCE_TEST_VERSION="${PERF_TEST_VERSION}")

if(PERFORMANCE_TEST_RCLCPP_ENABLED)
  if($ENV{ROS_DISTRO} MATCHES "humble")
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(${EXE_NAME} "${cpp_typesupport_target}")
  elseif($ENV{ROS_DISTRO} MATCHES "galactic")
    rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  elseif($ENV{ROS_DISTRO} MATCHES "foxy")
    rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  elseif($ENV{ROS_DISTRO} MATCHES "eloquent")
    rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  elseif($ENV{ROS_DISTRO} MATCHES "dashing")
    rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  elseif($ENV{ROS_DISTRO} MATCHES "rolling")
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(${EXE_NAME} "${cpp_typesupport_target}")
  else()
    message(FATAL_ERROR "Unsupported ROS_DISTRO")
  endif()
  ament_target_dependencies(${EXE_NAME} "rclcpp")
endif()

if(PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED)
  if(APEX_CERT)
    set(TYPESUPPORT_INTERFACE rosidl_typesupport_apex_middleware_cpp)
  else()
    set(TYPESUPPORT_INTERFACE rosidl_typesupport_cpp)
  endif()
  rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} ${TYPESUPPORT_INTERFACE})
  ament_target_dependencies(${EXE_NAME} "rclcpp")
endif()

ament_target_dependencies(${EXE_NAME} ${OPTIONAL_AMENT_DEPENDENCES})

target_link_libraries(
  ${EXE_NAME}
  ${OPTIONAL_LIBRARIES}
  ${PLUGIN_LIBRARIES}
  ${IDLGEN_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

if(BUILD_TESTING)
    find_package(ament_cmake_copyright REQUIRED)
    ament_copyright()

    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(
        ${sources})

    find_package(ament_cmake_cpplint REQUIRED)
    ament_cpplint(
        FILTERS "-build/include_order")

    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify(
        CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../ament_code_style.cfg"
        ${sources})

    find_package(ament_cmake_lint_cmake REQUIRED)
    ament_lint_cmake()

    set(APEX_PERFORMANCE_TEST_GTEST apex_performance_test_gtest)

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${APEX_PERFORMANCE_TEST_GTEST}
        test/src/test_performance_test.cpp
        test/src/test_statistics_tracker.hpp)

    target_include_directories(${APEX_PERFORMANCE_TEST_GTEST} PRIVATE "test/include")
    target_link_libraries(${APEX_PERFORMANCE_TEST_GTEST})

    set_compile_options(${APEX_PERFORMANCE_TEST_GTEST})
endif()

install(TARGETS
    ${EXE_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
    DESTINATION lib/${PROJECT_NAME})

install(FILES  mapping_rules.yaml
    DESTINATION share/${PROJECT_NAME})
ament_package()
