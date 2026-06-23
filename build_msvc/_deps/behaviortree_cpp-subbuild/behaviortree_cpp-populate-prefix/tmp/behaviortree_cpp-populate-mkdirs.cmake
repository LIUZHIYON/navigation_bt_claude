# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-src")
  file(MAKE_DIRECTORY "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-src")
endif()
file(MAKE_DIRECTORY
  "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-build"
  "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-subbuild/behaviortree_cpp-populate-prefix"
  "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-subbuild/behaviortree_cpp-populate-prefix/tmp"
  "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-subbuild/behaviortree_cpp-populate-prefix/src/behaviortree_cpp-populate-stamp"
  "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-subbuild/behaviortree_cpp-populate-prefix/src"
  "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-subbuild/behaviortree_cpp-populate-prefix/src/behaviortree_cpp-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-subbuild/behaviortree_cpp-populate-prefix/src/behaviortree_cpp-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/groot2_project/navigation_bt/build_msvc/_deps/behaviortree_cpp-subbuild/behaviortree_cpp-populate-prefix/src/behaviortree_cpp-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
