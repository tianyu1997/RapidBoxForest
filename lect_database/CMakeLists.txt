cmake_minimum_required(VERSION 3.20)
project(LECTDatabase VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(LECTDB_BUILD_TESTS "Build LECTDatabase C++ tests" ON)
option(LECTDB_BUILD_TOOLS "Build LECTDatabase command-line tools" ON)
set(LECTDB_ENVELOPE_MODULE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../link_interval_envelope"
    CACHE PATH "Path to the sibling link_interval_envelope module")

if(NOT EXISTS "${LECTDB_ENVELOPE_MODULE_DIR}/CMakeLists.txt"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../link-interval-envelope/CMakeLists.txt")
    set(LECTDB_ENVELOPE_MODULE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../link-interval-envelope"
        CACHE PATH "Path to the sibling link_interval_envelope module" FORCE)
endif()

find_package(Threads REQUIRED)

if(NOT TARGET link_interval_envelope::core AND NOT TARGET link_interval_envelope_core)
    find_package(LinkIntervalEnvelope QUIET)
endif()

if(NOT TARGET link_interval_envelope::core AND NOT TARGET link_interval_envelope_core)
    if(EXISTS "${LECTDB_ENVELOPE_MODULE_DIR}/CMakeLists.txt")
        set(LIE_WITH_PYTHON OFF CACHE BOOL "Disable envelope Python bindings for nested LECTDatabase builds" FORCE)
        set(LIE_BUILD_TESTS OFF CACHE BOOL "Disable envelope tests for nested LECTDatabase builds" FORCE)
        set(LIE_BUILD_EXAMPLES OFF CACHE BOOL "Disable envelope examples for nested LECTDatabase builds" FORCE)
        add_subdirectory(
            "${LECTDB_ENVELOPE_MODULE_DIR}"
            "${CMAKE_BINARY_DIR}/link_interval_envelope")
    else()
        message(FATAL_ERROR
            "link_interval_envelope::core was not found. Set "
            "LECTDB_ENVELOPE_MODULE_DIR to the sibling envelope module.")
    endif()
endif()

if(TARGET link_interval_envelope::core)
    set(LECTDB_ENVELOPE_TARGET link_interval_envelope::core)
else()
    set(LECTDB_ENVELOPE_TARGET link_interval_envelope_core)
endif()

set(LECTDB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(lect_database_core STATIC
    src/lect_database/types.cpp
    src/lect_database/split_policy.cpp
    src/lect_database/identity.cpp
    src/lect_database/canonicalization.cpp
    src/lect_database/database.cpp
    src/lect_database/database_evidence_index.cpp
    src/lect_database/database_file_layout.cpp
    src/lect_database/database_mapped_file.cpp
    src/lect_database/database_node_pages.cpp
    src/lect_database/read_snapshot.cpp
)
add_library(LECTDatabase::core ALIAS lect_database_core)

add_library(lect_database_sbf_adapter STATIC
    src/sbf/oracle.cpp
    src/sbf/scene.cpp
)
add_library(LECTDatabase::sbf_adapter ALIAS lect_database_sbf_adapter)

add_library(lect_database_online_cache STATIC
    src/online_cache/cache_tree.cpp
)
add_library(LECTDatabase::online_cache ALIAS lect_database_online_cache)

target_include_directories(lect_database_core PUBLIC
    $<BUILD_INTERFACE:${LECTDB_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lect_database_core PUBLIC
    ${LECTDB_ENVELOPE_TARGET}
    Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lect_database_core PRIVATE -Wall -Wextra -Wpedantic -O3)
endif()

target_include_directories(lect_database_sbf_adapter PUBLIC
    $<BUILD_INTERFACE:${LECTDB_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lect_database_sbf_adapter PUBLIC
    lect_database_core
    lect_database_online_cache
)

target_include_directories(lect_database_online_cache PUBLIC
    $<BUILD_INTERFACE:${LECTDB_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lect_database_online_cache PUBLIC
    lect_database_core
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lect_database_sbf_adapter PRIVATE -Wall -Wextra -Wpedantic -O3)
    target_compile_options(lect_database_online_cache PRIVATE -Wall -Wextra -Wpedantic -O3)
endif()

if(LECTDB_BUILD_TOOLS)
    add_executable(lect_database_tool tools/lect_database_tool.cpp)
    target_link_libraries(lect_database_tool PRIVATE lect_database_core)

    add_executable(lect_database_benchmark tools/lect_database_benchmark.cpp)
    target_link_libraries(lect_database_benchmark PRIVATE lect_database_core)
endif()

if(LECTDB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS lect_database_core lect_database_sbf_adapter lect_database_online_cache
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
