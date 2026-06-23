cmake_minimum_required(VERSION 3.16)
project(NavigationBT VERSION 1.0.0 LANGUAGES CXX)

# C++17 required for std::optional and structured bindings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# MSVC: treat all source as UTF-8 (fixes lexy raw string encoding errors)
if(MSVC)
    add_compile_options(/utf-8)
endif()

# ============================================================
# BehaviorTree.CPP - Fetch or find
# ============================================================
include(FetchContent)

FetchContent_Declare(
    behaviortree_cpp
    GIT_REPOSITORY  https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG         4.0.2
)
FetchContent_MakeAvailable(behaviortree_cpp)

# ============================================================
# Find packages
# ============================================================
find_package(OpenMP QUIET)

# ============================================================
# Sources
# ============================================================
set(CORE_HEADERS
    include/core/types.h
    include/core/grid_map.h
)

set(CORE_SOURCES
    src/core/grid_map.cpp
)

set(ALGORITHM_HEADERS
    include/algorithms/astar.h
)

set(ALGORITHM_SOURCES
    src/algorithms/astar.cpp
)

set(BT_NODE_HEADERS
    include/bt_nodes/navigation_nodes.h
    include/bt_nodes/condition_nodes.h
)

set(BT_NODE_SOURCES
    src/bt_nodes/navigation_nodes.cpp
    src/bt_nodes/condition_nodes.cpp
)

# ============================================================
# Shared library (for Groot2 runtime loading)
# ============================================================
add_library(navigation_bt_nodes SHARED
    ${BT_NODE_SOURCES}
    ${ALGORITHM_SOURCES}
    ${CORE_SOURCES}
)

target_include_directories(navigation_bt_nodes
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(navigation_bt_nodes
    PUBLIC
        behaviortree_cpp
)

# ============================================================
# Main executable (standalone demo)
# ============================================================
add_executable(navigation_demo
    src/main.cpp
    ${BT_NODE_SOURCES}
    ${ALGORITHM_SOURCES}
    ${CORE_SOURCES}
)

target_include_directories(navigation_demo
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(navigation_demo
    PRIVATE
        behaviortree_cpp
)

# ZMQ is bundled with BehaviorTree.CPP v4.x via FetchContent
# Use the same ZMQ targets that BehaviorTree.CPP builds
if(TARGET minizip)
    # BehaviorTree.CPP v4 fetches cppzmq/zmq internally
endif()

# Enable the ZMQ publisher for Groot2 real-time monitoring
target_compile_definitions(navigation_demo PRIVATE BT_ENABLE_ZMQ_PUBLISHER)

# OpenMP for parallel A* if available
if(OpenMP_FOUND)
    target_link_libraries(navigation_demo PRIVATE OpenMP::OpenMP_CXX)
endif()

# ============================================================
# Install
# ============================================================
install(TARGETS navigation_bt_nodes navigation_demo
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

install(FILES
    trees/navigation_main.xml
    trees/pathfinding_subtree.xml
    trees/obstacle_avoidance_subtree.xml
    DESTINATION share/navigation_bt/trees
)

# ============================================================
# Print summary
# ============================================================
message(STATUS "=== Navigation Behavior Tree Project ===")
message(STATUS "  Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "  Shared library:    navigation_bt_nodes")
message(STATUS "  Executable:        navigation_demo")
