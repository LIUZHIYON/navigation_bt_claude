# Install script for directory: E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/NavigationBT")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/Debug/behaviortree_cppd.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/Release/behaviortree_cpp.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/MinSizeRel/behaviortree_cpp.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/RelWithDebInfo/behaviortree_cpp.lib")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/Debug/behaviortree_cppd.dll")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/Release/behaviortree_cpp.dll")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/MinSizeRel/behaviortree_cpp.dll")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/RelWithDebInfo/behaviortree_cpp.dll")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-src/include/" FILES_MATCHING REGEX "/[^/]*\\.h[^/]*$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp/behaviortree_cppTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp/behaviortree_cppTargets.cmake"
         "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/CMakeFiles/Export/705a965934bb17904a36cf4506cbd489/behaviortree_cppTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp/behaviortree_cppTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp/behaviortree_cppTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp" TYPE FILE FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/CMakeFiles/Export/705a965934bb17904a36cf4506cbd489/behaviortree_cppTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp" TYPE FILE FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/CMakeFiles/Export/705a965934bb17904a36cf4506cbd489/behaviortree_cppTargets-debug.cmake")
  endif()
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp" TYPE FILE FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/CMakeFiles/Export/705a965934bb17904a36cf4506cbd489/behaviortree_cppTargets-minsizerel.cmake")
  endif()
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp" TYPE FILE FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/CMakeFiles/Export/705a965934bb17904a36cf4506cbd489/behaviortree_cppTargets-relwithdebinfo.cmake")
  endif()
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp" TYPE FILE FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/CMakeFiles/Export/705a965934bb17904a36cf4506cbd489/behaviortree_cppTargets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/behaviortree_cpp" TYPE FILE FILES "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/behaviortree_cppConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("E:/groot2_project/navigation_bt/build_msvc/_deps/googletest-build/cmake_install.cmake")
  include("E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/3rdparty/lexy/cmake_install.cmake")
  include("E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/sample_nodes/cmake_install.cmake")
  include("E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/tests/cmake_install.cmake")
  include("E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/tools/cmake_install.cmake")
  include("E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/examples/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
