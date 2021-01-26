
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
find_package(Boost COMPONENTS program_options timer REQUIRED)
find_package(ament_cmake REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(PERFORMANCE_TEST_RT_ENABLED_DEFAULT ON)
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

find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rcutils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(osrf_testing_tools_cpp QUIET)
if(${osrf_testing_tools_cpp_FOUND})
    list(APPEND OPTIONAL_AMENT_DEPENDENCES "osrf_testing_tools_cpp")
    list(APPEND OPTIONAL_LIBRARIES osrf_testing_tools_cpp::memory_tools)
    add_definitions(-DPERFORMANCE_TEST_MEMORYTOOLS_ENABLED)
endif()

# Dashing version looks like 0.7.X.
if(${rclcpp_VERSION_MAJOR} LESS 1 AND ${rclcpp_VERSION_MINOR} LESS 8)
  message(STATUS "ROS Dashing detected")
  add_definitions(-DUSE_LEGACY_QOS_API)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

# This is a workaround for broken include paths on some systems.
include_directories(${FastRTPS_INCLUDE_DIR} ${FastRTPS_INCLUDE_DIR}/fastrtps/include ${fastcdr_INCLUDE_DIR})
include_directories(include ${osrf_testing_tools_cpp_INCLUDE_DIR})

# FastRTPS
option(PERFORMANCE_TEST_FASTRTPS_ENABLED "Enable FastRTPS" OFF)

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  find_package(fastrtps_cmake_module QUIET REQUIRED)
  find_package(FastRTPS QUIET REQUIRED)
  add_definitions(-DPERFORMANCE_TEST_FASTRTPS_ENABLED)
  add_subdirectory(src/idlgen/fast_rtps)
  include_directories(${FAST_RTPS_IDL_INCLUDE_DIR})
endif()


if(PERFORMANCE_TEST_CONNEXTDDS_ENABLED AND PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
  message(FATAL_ERROR
    "ConnextDDS Pro and ConnextDDS Micro cannot be used in the same application. Please, disable one")
endif()

# ConnextDDS
option(PERFORMANCE_TEST_CONNEXTDDS_ENABLED
  "Option to enable Connext DDS Pro"
  OFF
)

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
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/idlgen/connextdds")
endif()


# RTI Connext Micro
option(PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED "Option to enable Connext DDS Micro plugin. The
plugin can only work if ApexOS is present with RTI Connext Micro, otherwise it will fail." OFF)

if(PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
  find_package(connext_micro_cmake_module REQUIRED)
  find_package(ConnextMicro MODULE REQUIRED)
  if(ConnextMicro_FOUND)
    add_definitions(-DPERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
    include_directories(${RTI_MICRO_IDL_INCLUDE_DIR})
    ament_export_definitions(${ConnextMicro_DEFINITIONS})
  endif()
endif()


# CycloneDDS
option(PERFORMANCE_TEST_CYCLONEDDS_ENABLED "Enable CycloneDDS" OFF)
if(PERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  find_package(CycloneDDS REQUIRED)
  add_definitions(-DPERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  add_subdirectory(src/idlgen/cyclonedds)
  include_directories(${CYCLONEDDS_IDL_INCLUDE_DIR})
endif()

# ROS2 Callback and Executor
option(PERFORMANCE_TEST_CALLBACK_EXECUTOR_ENABLED "Option to enable the Callback and Executor Plugin." ON)

# Apex.OS Waitset and Polling Subscription
option(PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED "Option to enable Polling Subscription Plugin.
  The plugin can only work if ApexOS is present, otherwise it will fail." OFF)

# OpenDDS
option(PERFORMANCE_TEST_OPENDDS_ENABLED "Require Open DDS to be available, fail otherwise" OFF)
if(PERFORMANCE_TEST_OPENDDS_ENABLED)
  add_definitions(-DPERFORMANCE_TEST_OPENDDS_ENABLED)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  add_subdirectory(src/idlgen/opendds)
  include_directories(${OPENDDS_IDL_INCLUDE_DIR} ${PARENT_DDS_ROOT_INCLUDE})
endif()

set(${PROJECT_NAME}_SOURCES
    src/experiment_execution/analyze_runner.cpp)

set(${PROJECT_NAME}_HEADERS
    src/experiment_configuration/experiment_configuration.hpp
    src/experiment_execution/analysis_result.hpp)

option(IDL_MESSAGES_ENABLED "Use the .idl files in the src/msg folder, instead of the .msg files" OFF)

if(IDL_MESSAGES_ENABLED)
  set(MESSAGE_FILE_EXTENSION "idl")
else()
  set(MESSAGE_FILE_EXTENSION "msg")
endif()

set(MESSAGE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array1k.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array4k.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array16k.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array32k.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array60k.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array1m.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array2m.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array4m.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array8m.${MESSAGE_FILE_EXTENSION}"

    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct16.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct256.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct4k.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct32k.${MESSAGE_FILE_EXTENSION}"

    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud1m.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud2m.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud4m.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud8m.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud512k.${MESSAGE_FILE_EXTENSION}"

    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/NavSatStatus.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/NavSatFix.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/RadarDetection.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/RadarTrack.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Range.${MESSAGE_FILE_EXTENSION}"

    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Point.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Point32.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Polygon.${MESSAGE_FILE_EXTENSION}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Vector3.${MESSAGE_FILE_EXTENSION}")

option(PERFORMANCE_TEST_ZERO_COPY_ENABLED "Enable the option to use zero copy transfer" OFF)

if(PERFORMANCE_TEST_ZERO_COPY_ENABLED)
  if(PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED)
    add_definitions(-DPERFORMANCE_TEST_ZERO_COPY_ENABLED)
    set(MESSAGE_FILES_ZC
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Header.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointField.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array1kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array4kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array16kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array32kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array60kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array1mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array2mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array4mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Array8mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct16ZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct256ZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct4kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Struct32kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud1mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud2mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud4mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud8mZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointCloud512kZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/NavSatStatusZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/NavSatFixZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/RadarDetectionZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/RadarTrackZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/RangeZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PointZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Point32ZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/PolygonZeroCopy.idl"
        "${CMAKE_CURRENT_SOURCE_DIR}/src:msg/Vector3ZeroCopy.idl")
  else()
    message("Error: Zero copy requires polling subscription and Apex.OS with RTI Connext Micro. Zero copy will not be enabled.")
  endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MESSAGE_FILES}
    ${MESSAGE_FILES_ZC}
    DEPENDENCIES builtin_interfaces std_msgs sensor_msgs)

set(sources
    src/main.cpp
    src/communication_abstractions/communicator.hpp
    src/communication_abstractions/communicator.cpp
    src/communication_abstractions/resource_manager.cpp
    src/communication_abstractions/resource_manager.hpp
    src/experiment_configuration/topics.hpp
    src/data_running/data_runner.hpp
    src/data_running/data_runner_base.hpp
    src/data_running/data_runner_factory.cpp
    src/data_running/data_runner_factory.hpp
    src/experiment_execution/analyze_runner.hpp
    src/experiment_execution/analysis_result.cpp
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
)

if(PERFORMANCE_TEST_RT_ENABLED)
  add_definitions(-DPERFORMANCE_TEST_RT_ENABLED)
  list(APPEND sources src/utilities/rt_enabler.hpp)
endif()

if(PERFORMANCE_TEST_CALLBACK_EXECUTOR_ENABLED)
  add_definitions(-DPERFORMANCE_TEST_CALLBACK_EXECUTOR_ENABLED)
  list(APPEND sources src/communication_abstractions/ros2_callback_communicator.hpp)
endif()

if(PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED)
  add_definitions(-DPERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED)
  list(APPEND sources src/communication_abstractions/ros2_waitset_communicator.hpp)
endif()

if(PERFORMANCE_TEST_CALLBACK_EXECUTOR_ENABLED OR PERFORMANCE_TEST_POLLING_SUBSCRIPTION_ENABLED)
  list(APPEND sources src/communication_abstractions/ros2_communicator.hpp)
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

if(PERFORMANCE_TEST_OPENDDS_ENABLED)
  list(APPEND sources src/communication_abstractions/opendds_communicator.hpp)
endif()

# Use ODB support to create SQL database
option(PERFORMANCE_TEST_ODB_SQLITE "Enable ODB with SQLITE" OFF)
option(PERFORMANCE_TEST_ODB_MYSQL "Enable ODB with MYSQL" OFF)
option(PERFORMANCE_TEST_ODB_PGSQL "Enable ODB with PGSQL" OFF)
if(PERFORMANCE_TEST_ODB_SQLITE OR PERFORMANCE_TEST_ODB_MYSQL OR PERFORMANCE_TEST_ODB_PGSQL)
  if(PERFORMANCE_TEST_ODB_SQLITE)
    set(DB_TYPE sqlite)
  elseif(PERFORMANCE_TEST_ODB_MYSQL)
    set(DB_TYPE mysql)
  elseif(PERFORMANCE_TEST_ODB_PGSQL)
    set(DB_TYPE pgsql)
  endif()

  set(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED ON)
  add_definitions(-DPERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
  find_package(ODB REQUIRED COMPONENTS sqlite OPTIONAL_COMPONENTS mysql pgsql)
  include(${ODB_USE_FILE})

  # Get the setup.sh to update the LD_LIBRARY_PATH
  get_filename_component(ODB_LIBRARY_DIRECTORY ${ODB_LIBODB_LIBRARIES} DIRECTORY)
  ament_environment_hooks(env_hook/odb_library_path.sh.in)

  odb_compile(${PROJECT_NAME}_SOURCES FILES ${${PROJECT_NAME}_HEADERS} DB "${DB_TYPE}"
      GENERATE_QUERY GENERATE_SESSION GENERATE_SCHEMA SCHEMA_FORMAT embedded CHANGELOG_DIR
      ${PROJECT_SOURCE_DIR}/schema_changelog INCLUDE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_configuration
      ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_execution
      STANDARD c++11 PROFILE boost/uuid X_CPP_OPTION -DPERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)

  include_directories(${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_configuration
      ${CMAKE_CURRENT_SOURCE_DIR}/src/experiment_execution
      ${ODB_INCLUDE_DIRS})
endif()

set(EXE_NAME perf_test)
add_executable(${EXE_NAME} ${${PROJECT_NAME}_SOURCES} ${sources} ${${PROJECT_NAME}_HEADERS})


set_compile_options(${EXE_NAME})


# Try to bake the githash into the perf_test EXE:
execute_process(
  COMMAND ./version_check.bash
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE PERF_TEST_VERSION
)
add_definitions(-DPERFORMANCE_TEST_VERSION="${PERF_TEST_VERSION}")

rosidl_target_interfaces(${EXE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(${EXE_NAME}
    "rclcpp" ${OPTIONAL_AMENT_DEPENDENCES})

# If the target exists, it means we want to build the performance test working
# with the Connext DDS Communicator.
if(TARGET rti_connextdds_idl)
  # Link against the Connext DDS IDL types library
  target_link_libraries(${EXE_NAME}
    rti_connextdds_idl
  )

  # Add the definition to enable the Connext DDS Communicator
  target_compile_definitions(${EXE_NAME}
    PRIVATE
      PERFORMANCE_TEST_CONNEXTDDS_ENABLED
  )
endif()

target_link_libraries(${EXE_NAME}
    ${Boost_LIBRARIES}
    ${OPTIONAL_LIBRARIES}
)

if(PERFORMANCE_TEST_ODB_FOR_SQL_ENABLED)
  foreach(header ${${PROJECT_NAME}_HEADERS})
    get_filename_component(target_suffix ${header} NAME)
    add_dependencies(${EXE_NAME} "generate_odb_${target_suffix}")
  endforeach()
  target_link_libraries(${EXE_NAME} ${ODB_LIBRARIES})
  target_include_directories(${EXE_NAME}
      PRIVATE
      ${ODB_INCLUDE_DIRS}
      ${ODB_COMPILE_OUTPUT_DIR})
  if(PERFORMANCE_TEST_ODB_SQLITE)
    target_compile_definitions(${EXE_NAME}
      PRIVATE
      DATABASE_SQLITE)
  elseif(PERFORMANCE_TEST_ODB_MYSQL)
    target_compile_definitions(${EXE_NAME}
      PRIVATE
      DATABASE_MYSQL)
  elseif(PERFORMANCE_TEST_ODB_PGSQL)
    target_compile_definitions(${EXE_NAME}
      PRIVATE
      DATABASE_PGSQL)
  endif()
endif()

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  target_link_libraries(${EXE_NAME}
      ${FastRTPS_LIBRARIES}
      fast_rtps_idl
      fastcdr)
endif()

if(PERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  target_link_libraries(${EXE_NAME}
      cyclonedds_idl
      CycloneDDS::ddsc)
endif()
if(PERFORMANCE_TEST_OPENDDS_ENABLED)
    set(DDS_DYN_LIBS  ${PARENT_DDS_ROOT_LIBS}/libACE.so
                      ${PARENT_DDS_ROOT_LIBS}/libTAO.so
                      ${PARENT_DDS_ROOT_LIBS}/libOpenDDS_Dcps.so
                      ${PARENT_DDS_ROOT_LIBS}/libOpenDDS_Rtps.so
                      ${PARENT_DDS_ROOT_LIBS}/libOpenDDS_Rtps_Udp.so
                      ${PARENT_DDS_ROOT_LIBS}/libTAO_PortableServer.so
                      ${PARENT_DDS_ROOT_LIBS}/libTAO_PortableServer.so
                      ${PARENT_DDS_ROOT_LIBS}/libTAO_PI.so
                      ${PARENT_DDS_ROOT_LIBS}/libTAO_CodecFactory.so
                      ${PARENT_DDS_ROOT_LIBS}/libTAO_AnyTypeCode.so
                      ${PARENT_DDS_ROOT_LIBS}/libTAO_BiDirGIOP.so)

    target_link_libraries(${EXE_NAME} opendds_idl ${DDS_DYN_LIBS})
endif()

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
