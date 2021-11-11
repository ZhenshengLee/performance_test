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

set(SOURCE_EXTENSION cxx)
find_package(rmw REQUIRED)
find_package(FastRTPS REQUIRED)
include_directories(${FastRTPS_INCLUDE_DIR} ${fastcdr_INCLUDE_DIR})
# This is a workaround for broken include paths on some systems.
include_directories(${FastRTPS_INCLUDE_DIR}/fastrtps/include ${fastcdr_INCLUDE_DIR})
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/fast_rtps)
file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})

find_program(FASTRTPS_BIN "fastrtpsgen")
if(NOT FASTRTPS_BIN)
  message(WARNING "Could not find program 'fastrtpsgen' in path. Trying to use bundled version")
  find_program(FASTRTPS_BIN "fastrtpsgen" PATHS "${PROJECT_SOURCE_DIR}/bin")
  if(NOT FASTRTPS_BIN)
    message(FATAL_ERROR "Could not find program 'fastrtpsgen'")
  endif()
endif()

foreach(idl ${IDL_FILES})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})

  file(READ ${idl} filedata)
  string(REGEX REPLACE "__plugin__" "FastDDSGen" filedata "${filedata}")
  file(WRITE ${IDL_GEN_ROOT}/${filename} "${filedata}")

  set(IDL_GEN_C ${IDL_GEN_ROOT}/${basename}.${SOURCE_EXTENSION}
      ${IDL_GEN_ROOT}/${basename}PubSubTypes.${SOURCE_EXTENSION})

  set(IDL_GEN_H ${IDL_GEN_ROOT}/${basename}.h
      ${IDL_GEN_ROOT}/${basename}PubSubTypes.h)

  set(IDL_GEN_SS ${IDL_GEN_ROOT}/${basename}PubSubTypes.h)

  add_custom_command(OUTPUT ${IDL_GEN_SS}
      COMMAND ${FASTRTPS_BIN}
      -replace
      -d ${IDL_GEN_ROOT}
      ${IDL_GEN_ROOT}/${filename}
      DEPENDS ${IDL_GEN_ROOT}/${filename}
      WORKING_DIRECTORY ${IDL_GEN_ROOT}
      COMMENT "Regeneration type support plugin from ${idl}" VERBATIM)
  set_source_files_properties(${IDL_GEN_C} ${IDL_GEN_H} PROPERTIES GENERATED 1)

  list(APPEND IDL_GEN_H_LIST ${IDL_GEN_H})
  list(APPEND IDL_GEN_C_LIST ${IDL_GEN_C})

endforeach()

add_library(fast_rtps_idl
    ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST} ${FastRTPS_LIBRARIES})
set_compile_options(fast_rtps_idl)
set(FAST_RTPS_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
