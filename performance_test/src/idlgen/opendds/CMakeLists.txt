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

set(SOURCE_EXTENSION cpp)

set(DDS_ROOT /opt/opendds/share/dds)
set(ACE_ROOT ${DDS_ROOT}/../ace)
set(MPC_ROOT ${DDS_ROOT}/MPC)
set(TAO_ROOT ${DDS_ROOT}/../tao)

set(TAO_IDL DDS_ROOT=${DDS_ROOT} ACE_ROOT=${ACE_ROOT} TAO_ROOT=${TAO_ROOT} MPC_ROOT=${MPC_ROOT} LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):${ACE_ROOT}/lib:${DDS_ROOT}/lib ${ACE_ROOT}/bin/tao_idl)
set(OPENDDS_IDL DDS_ROOT=${DDS_ROOT} ACE_ROOT=${ACE_ROOT} TAO_ROOT=${TAO_ROOT} MPC_ROOT=${MPC_ROOT} LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):${ACE_ROOT}/lib:${DDS_ROOT}/lib ${DDS_ROOT}/bin/opendds_idl)
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

set(IDL_SOURCE_C
  ../Array1k_.idl
  ../Array4k_.idl
  ../Array16k_.idl
  ../Array32k_.idl
  ../Array60k_.idl
  ../Array1m_.idl
  ../Array2m_.idl
  ../Array4m_.idl
  ../Array8m_.idl
  ../Struct16_.idl
  ../Struct256_.idl
  ../Struct4k_.idl
  ../Struct32k_.idl
  ../PointField_.idl
  ../Header_.idl
  ../Time_.idl
  ../PointCloud512k_.idl
  ../PointCloud1m_.idl
  ../PointCloud2m_.idl
  ../PointCloud4m_.idl
  ../PointCloud8m_.idl
  ../Range_.idl
  ../NavSatStatus_.idl
  ../Point_.idl
  ../Point32_.idl
  ../Vector3_.idl
  ../Polygon_.idl
  ../NavSatFix_.idl
  ../RadarDetection_.idl
  ../RadarTrack_.idl
)

set(IDL_GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen/opendds)

file(MAKE_DIRECTORY ${IDL_GEN_ROOT})

if(${CMAKE_BUILD_TYPE} MATCHES "[Rr]elease")
  add_definitions(-DNDEBUG)
endif()

link_directories(${DDS_ROOT}/../../lib)
include_directories(${IDL_GEN_ROOT} ${DDS_ROOT}/../../include/)

foreach(idl ${IDL_SOURCE_C})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})
  set(TAO_IDL_GEN_CPP ${IDL_GEN_ROOT}/${basename}C.${SOURCE_EXTENSION}
          ${IDL_GEN_ROOT}/${basename}S.${SOURCE_EXTENSION}
          ${IDL_GEN_ROOT}/${basename}C.inl)

  set(TAO_IDL_GEN_H ${IDL_GEN_ROOT}/${basename}C.h
          ${IDL_GEN_ROOT}/${basename}S.h)

  set(TAO_IDL_GEN_CPP_SND_STG ${IDL_GEN_ROOT}/${basename}TypeSupportC.${SOURCE_EXTENSION}
                                    ${IDL_GEN_ROOT}/${basename}TypeSupportS.${SOURCE_EXTENSION}
                                    ${IDL_GEN_ROOT}/${basename}TypeSupportC.inl)

  set(TAO_IDL_GEN_H_SCD_STG  ${IDL_GEN_ROOT}/${basename}TypeSupportC.h
                                  ${IDL_GEN_ROOT}/${basename}TypeSupportS.h)

  set(OPENDDS_IDL_GEN_CPP ${IDL_GEN_ROOT}/${basename}TypeSupportImpl.${SOURCE_EXTENSION})

  set(OPENDDS_IDL_GEN_H ${IDL_GEN_ROOT}/${basename}TypeSupportImpl.h)

  add_custom_command(OUTPUT ${TAO_IDL_GEN_CPP} ${TAO_IDL_GEN_H}
          COMMAND ${TAO_IDL} -Cw -Sa -St -o ${IDL_GEN_ROOT}/. -I${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          COMMENT "Regeneration type support plugin from ${idl}")

  add_custom_command(OUTPUT ${OPENDDS_IDL_GEN_CPP}
          ${OPENDDS_IDL_GEN_H} ${IDL_GEN_ROOT}/${basename}TypeSupport.idl
          COMMAND ${OPENDDS_IDL} -Cw -Sa -St -o ${IDL_GEN_ROOT}/. -I${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${idl}
          COMMENT "Regeneration type support plugin from ${idl}")

  add_custom_command(OUTPUT ${TAO_IDL_GEN_CPP_SND_STG} ${TAO_IDL_GEN_H_SCD_STG}
          COMMAND ${TAO_IDL} -Cw -Sa -St -o ${IDL_GEN_ROOT}/. -I${CMAKE_CURRENT_SOURCE_DIR}/.. -I${DDS_ROOT}/../../include/tao -I${DDS_ROOT}/../../include/ ${IDL_GEN_ROOT}/${basename}TypeSupport.idl
          DEPENDS ${IDL_GEN_ROOT}/${basename}TypeSupport.idl
          COMMENT "Regeneration type support plugin from ${idl}")

  set_source_files_properties(${TAO_IDL_GEN_CPP} ${TAO_IDL_GEN_H} PROPERTIES GENERATED 1)
  set_source_files_properties(${OPENDDS_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_H} PROPERTIES GENERATED 1)
  set_source_files_properties(${TAO_IDL_GEN_CPP_SND_STG} ${TAO_IDL_GEN_H_SCD_STG} PROPERTIES GENERATED 1)
  list(APPEND IDL_GEN_H_LIST ${TAO_IDL_GEN_H} ${OPENDDS_IDL_GEN_H} ${TAO_IDL_GEN_H_SCD_STG})
  list(APPEND IDL_GEN_C_LIST ${TAO_IDL_GEN_CPP} ${OPENDDS_IDL_GEN_CPP} ${TAO_IDL_GEN_CPP_SND_STG})

endforeach()

add_library(opendds_idl
        ${IDL_GEN_H_LIST} ${IDL_GEN_C_LIST})
set_compile_options(opendds_idl)
set(OPENDDS_IDL_INCLUDE_DIR ${IDL_GEN_ROOT}/.. PARENT_SCOPE)
set(PARENT_DDS_ROOT_INCLUDE ${DDS_ROOT}/../../include PARENT_SCOPE)
set(PARENT_DDS_ROOT_LIBS ${DDS_ROOT}/../../lib PARENT_SCOPE)
