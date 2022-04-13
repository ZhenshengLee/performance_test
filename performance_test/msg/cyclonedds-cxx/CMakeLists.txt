# Copyright 2021 Apex.AI, Inc.
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

set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/cyclonedds-cxx)

file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

foreach(idl ${IDL_FILES})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})

  file(READ ${idl} filedata)
  string(REGEX REPLACE "__plugin__" "CycloneDDSCXXGen" filedata "${filedata}")
  file(WRITE ${IDL_GEN_ROOT}/${filename} "${filedata}")


  set(IDL_GEN_HPP ${IDL_GEN_ROOT}/${basename}.hpp)
  set(IDL_GEN_CPP ${IDL_GEN_ROOT}/${basename}.cpp)

  add_custom_command(
    OUTPUT   ${IDL_GEN_HPP} ${IDL_GEN_CPP}
    COMMAND  CycloneDDS::idlc
    ARGS     -l $<TARGET_FILE:CycloneDDS-CXX::idlcxx> -fcase-sensitive -fkeylist ${IDL_GEN_ROOT}/${filename}
    DEPENDS  ${IDL_GEN_ROOT}/${filename} CycloneDDS::idlc CycloneDDS-CXX::idlcxx
    COMMENT  "Generating C++ typesupport for ${idl}"
    WORKING_DIRECTORY ${IDL_GEN_ROOT}
    VERBATIM)
  set_source_files_properties(${IDL_GEN_HPP} ${IDL_GEN_CPP} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_HPP_LIST ${IDL_GEN_HPP})
  list(APPEND IDL_GEN_CPP_LIST ${IDL_GEN_CPP})
endforeach()

add_custom_target(cyclonedds_cxx_idl_generate DEPENDS ${IDL_GEN_HPP_LIST})
add_library(cyclonedds_cxx_idl STATIC ${IDL_GEN_HPP_LIST} ${IDL_GEN_CPP_LIST})
target_link_libraries(cyclonedds_cxx_idl CycloneDDS-CXX::ddscxx)
target_include_directories(cyclonedds_cxx_idl INTERFACE ${IDL_GEN_ROOT}/..)
add_dependencies(cyclonedds_cxx_idl cyclonedds_cxx_idl_generate)

set(CYCLONEDDS_CXX_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
