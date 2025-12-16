#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qcar2_interfaces::qcar2_interfaces__rosidl_typesupport_fastrtps_cpp" for configuration "Release"
set_property(TARGET qcar2_interfaces::qcar2_interfaces__rosidl_typesupport_fastrtps_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(qcar2_interfaces::qcar2_interfaces__rosidl_typesupport_fastrtps_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libqcar2_interfaces__rosidl_typesupport_fastrtps_cpp.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libqcar2_interfaces__rosidl_typesupport_fastrtps_cpp.dylib"
  )

list(APPEND _cmake_import_check_targets qcar2_interfaces::qcar2_interfaces__rosidl_typesupport_fastrtps_cpp )
list(APPEND _cmake_import_check_files_for_qcar2_interfaces::qcar2_interfaces__rosidl_typesupport_fastrtps_cpp "${_IMPORT_PREFIX}/lib/libqcar2_interfaces__rosidl_typesupport_fastrtps_cpp.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
