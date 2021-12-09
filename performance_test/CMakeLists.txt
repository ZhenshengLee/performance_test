
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

if(PERFORMANCE_TEST_RCLCPP_ENABLED)
  # rosidl_generate_interfaces must be invoked from this CMakeLists.txt file. Otherwise, the
  # generated messages are placed in a directory where ros1_bridge can not find them.
  rosidl_generate_interfaces(${PROJECT_NAME} ${ROSIDL_GEN_LIST})
endif()


set(${PROJECT_NAME}_SOURCES
    src/experiment_execution/analyze_runner.cpp)

set(${PROJECT_NAME}_HEADERS
    src/experiment_configuration/experiment_configuration.hpp
    src/experiment_execution/analysis_result.hpp)

set(sources
    src/main.cpp
    src/communication_abstractions/communicator.hpp
    src/communication_abstractions/communicator.cpp
    src/communication_abstractions/resource_manager.cpp
    src/communication_abstractions/resource_manager.hpp
    src/outputs/stdout_output.cpp
    src/outputs/csv_output.cpp
    src/outputs/json_output.cpp
    src/data_running/data_runner.hpp
    src/data_running/data_runner_base.hpp
    src/data_running/data_runner_factory.cpp
    src/data_running/data_runner_factory.hpp
    src/experiment_execution/analyze_runner.hpp
    src/experiment_execution/analysis_result.cpp
    src/experiment_configuration/communication_mean.cpp
    src/experiment_configuration/communication_mean.hpp
    src/experiment_configuration/qos_abstraction.cpp
    src/experiment_configuration/qos_abstraction.hpp
    src/experiment_configuration/experiment_configuration.cpp
    src/experiment_configuration/external_info_storage.hpp
    src/experiment_configuration/external_info_storage.cpp
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
  list(APPEND sources src/communication_abstractions/rclcpp_waitset_communicator.hpp)
  list(APPEND sources src/data_running/factories/rclcpp_ste_data_runner_factory.cpp)
  list(APPEND sources src/data_running/factories/rclcpp_ste_data_runner_factory.hpp)
  list(APPEND sources src/data_running/factories/rclcpp_sste_data_runner_factory.cpp)
  list(APPEND sources src/data_running/factories/rclcpp_sste_data_runner_factory.hpp)
  list(APPEND sources src/data_running/factories/rclcpp_waitset_data_runner_factory.cpp)
  list(APPEND sources src/data_running/factories/rclcpp_waitset_data_runner_factory.hpp)
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
  PATCH_COMMAND patch ${EXTERNAL_INSTALL_LOCATION}/sole/sole.hpp ${CMAKE_CURRENT_SOURCE_DIR}/external_patches/sole.hpp.patch
  CMAKE_COMMAND ""
  CONFIGURE_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_COMMAND ""
)
include_directories(SYSTEM ${EXTERNAL_INSTALL_LOCATION})

set(EXE_NAME perf_test)
add_executable(${EXE_NAME} ${${PROJECT_NAME}_SOURCES} ${sources} ${${PROJECT_NAME}_HEADERS})
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
  rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
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
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_copyright(${${PROJECT_NAME}_SOURCES} ${sources} ${${PROJECT_NAME}_HEADERS})

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
