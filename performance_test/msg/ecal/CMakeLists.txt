# Copyright 2022 Zhenshenglee, Inc.
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

set(SOURCE_EXTENSION cc)
find_package(rmw REQUIRED)
find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

# colcon/build/performance_test/msg/fast_rtps/gen/ecal
set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/ecal)
file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/objs/${RTIME_TARGET_NAME})

foreach(idl ${PROTO_FILES})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.proto" "" basename ${filename})

  file(READ ${idl} filedata)
  string(REGEX REPLACE "__plugin__" "EcalGen" filedata "${filedata}")
  file(WRITE ${IDL_GEN_ROOT}/${filename} "${filedata}")

  set(IDL_GEN_C ${IDL_GEN_ROOT}/${basename}.pb.${SOURCE_EXTENSION})
  set(IDL_GEN_H ${IDL_GEN_ROOT}/${basename}.pb.h)

  LIST(APPEND PROTO_FLAGS -I${IDL_GEN_ROOT})

  add_custom_command(
    OUTPUT  ${IDL_GEN_C} ${IDL_GEN_H}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS    ${PROTO_FLAGS} --cpp_out=. ${IDL_GEN_ROOT}/${filename}
    DEPENDS ${IDL_GEN_ROOT}/${filename}
    WORKING_DIRECTORY ${IDL_GEN_ROOT}
    COMMENT "Running protoc on ${idl}" VERBATIM)
  set_source_files_properties(${IDL_GEN_C} ${IDL_GEN_H} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_H_LIST ${IDL_GEN_H})
  list(APPEND IDL_GEN_C_LIST ${IDL_GEN_C})

endforeach()

add_library(ecal_idl ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})
set_compile_options(ecal_idl)
set(ECAL_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
