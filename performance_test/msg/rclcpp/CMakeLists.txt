set(IDL_GEN_ROOT ${CMAKE_BINARY_DIR}/gen/rosidl)
file(MAKE_DIRECTORY "${IDL_GEN_ROOT}/msg")
foreach(idl ${IDL_FILES})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})
  set(output_file "${IDL_GEN_ROOT}/msg/${filename}")

  file(READ ${idl} filedata)
  string(REGEX REPLACE "module __plugin__ {" "" filedata "${filedata}")
  string(REGEX REPLACE "}; // __plugin__" "" filedata "${filedata}")
  string(REGEX REPLACE "__plugin__::" "" filedata "${filedata}")
  string(REGEX REPLACE "#include \"" "#include \"performance_test/msg/" filedata "${filedata}")
  file(WRITE ${output_file} "${filedata}")

  list(APPEND ROSIDL_GEN_LIST "${IDL_GEN_ROOT}:msg/${filename}")
endforeach()

# If we ever drop the ros1_bridge packages, just directly invoke
# rosidl_generate_interfaces(${PROJECT_NAME} ${ROSIDL_GEN_LIST})
# instead of bubbling the ROSIDL_GEN_LIST all the way up
set(ROSIDL_GEN_LIST ${ROSIDL_GEN_LIST} PARENT_SCOPE)
