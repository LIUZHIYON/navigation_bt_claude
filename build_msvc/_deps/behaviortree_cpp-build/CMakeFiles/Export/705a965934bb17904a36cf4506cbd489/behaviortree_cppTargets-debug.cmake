#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "BT::behaviortree_cpp" for configuration "Debug"
set_property(TARGET BT::behaviortree_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(BT::behaviortree_cpp PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/behaviortree_cppd.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "Boost::coroutine"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/behaviortree_cppd.dll"
  )

list(APPEND _cmake_import_check_targets BT::behaviortree_cpp )
list(APPEND _cmake_import_check_files_for_BT::behaviortree_cpp "${_IMPORT_PREFIX}/lib/behaviortree_cppd.lib" "${_IMPORT_PREFIX}/bin/behaviortree_cppd.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
