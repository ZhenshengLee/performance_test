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

set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/cyclonedds)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})

file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

foreach(idl ${IDL_FILES})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})

  file(READ ${idl} filedata)
  string(REGEX REPLACE "__plugin__" "CycloneDDSGen" filedata "${filedata}")
  file(WRITE ${IDL_GEN_ROOT}/${filename} "${filedata}")
  file(APPEND ${IDL_GEN_ROOT}/${filename}
    "#pragma keylist performance_test::msg::CycloneDDSGen::${basename}\n")


  set(IDL_GEN_C ${IDL_GEN_ROOT}/${basename}.c)
  set(IDL_GEN_H ${IDL_GEN_ROOT}/${basename}.h)

  add_custom_command(
    OUTPUT   ${IDL_GEN_C} ${IDL_GEN_H}
    COMMAND  CycloneDDS::idlc
    ARGS     -fcase-sensitive -fkeylist ${IDL_GEN_ROOT}/${filename}
    DEPENDS  ${IDL_GEN_ROOT}/${filename}
    COMMENT  "Running idlc on ${FIL}"
    WORKING_DIRECTORY ${IDL_GEN_ROOT}
    VERBATIM)
  set_source_files_properties(${IDL_GEN_C} ${IDL_GEN_H} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_H_LIST ${IDL_GEN_H})
  list(APPEND IDL_GEN_C_LIST ${IDL_GEN_C})
endforeach()

add_library(cyclonedds_idl ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})

get_target_property(cdds_include_dir CycloneDDS::ddsc INTERFACE_INCLUDE_DIRECTORIES)
include_directories("${cdds_include_dir}")

set_compile_options(cyclonedds_idl)
target_compile_options(cyclonedds_idl BEFORE PRIVATE -fvisibility=default)
set(CYCLONEDDS_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
