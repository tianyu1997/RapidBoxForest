cmake_minimum_required(VERSION 3.20)
project(SafeBoxForest VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SBF_BUILD_TESTS "Build SBF C++ tests" ON)
option(SBF_BUILD_EXPERIMENTS "Build SBF lightweight experiment drivers" OFF)
option(SBF_WITH_PYTHON "Build SBF Python extension" ON)
option(SBF_INSTALL_CMAKE_PACKAGE "Install SBF CMake package export files" OFF)
set(SBF_LECT_DATABASE_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lect_database" CACHE PATH "Path to the sibling lect_database module")

if(NOT EXISTS "${SBF_LECT_DATABASE_MODULE_DIR}/CMakeLists.txt"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../LECTDatabase/CMakeLists.txt")
    set(SBF_LECT_DATABASE_MODULE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../LECTDatabase"
        CACHE PATH "Path to the sibling lect_database module" FORCE)
endif()

if(NOT TARGET LECTDatabase::core AND NOT TARGET lect_database_core)
    find_package(LECTDatabase QUIET)
endif()

if(NOT TARGET LECTDatabase::core AND NOT TARGET lect_database_core)
    if(EXISTS "${SBF_LECT_DATABASE_MODULE_DIR}/CMakeLists.txt")
        set(LECTDB_BUILD_TESTS OFF CACHE BOOL "Disable LECTDatabase tests for nested SBF builds" FORCE)
        set(LECTDB_BUILD_TOOLS OFF CACHE BOOL "Disable LECTDatabase tools for nested SBF builds" FORCE)
        add_subdirectory("${SBF_LECT_DATABASE_MODULE_DIR}" "${CMAKE_BINARY_DIR}/lect_database")
    else()
        message(FATAL_ERROR "LECTDatabase::core was not found. Set SBF_LECT_DATABASE_MODULE_DIR to the sibling lect_database module.")
    endif()
endif()

if(TARGET LECTDatabase::core)
    set(SBF_LECTDB_CORE_TARGET LECTDatabase::core)
else()
    set(SBF_LECTDB_CORE_TARGET lect_database_core)
endif()

if(TARGET LECTDatabase::sbf_adapter)
    set(SBF_LECTDB_ADAPTER_TARGET LECTDatabase::sbf_adapter)
elseif(TARGET lect_database_sbf_adapter)
    set(SBF_LECTDB_ADAPTER_TARGET lect_database_sbf_adapter)
else()
    set(SBF_LECTDB_ADAPTER_TARGET "")
endif()

if(TARGET LECTDatabase::online_cache)
    set(SBF_LECTDB_CACHE_TARGET LECTDatabase::online_cache)
else()
    set(SBF_LECTDB_CACHE_TARGET lect_database_online_cache)
endif()

find_package(Threads REQUIRED)

if(SBF_WITH_PYTHON)
    include(FetchContent)
    set(PYBIND11_FINDPYTHON ON CACHE BOOL "Use CMake FindPython for pybind11" FORCE)
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(ompl REQUIRED HINTS /usr/share/ompl/cmake)
    find_package(pybind11 2.12 QUIET)
    if(NOT pybind11_FOUND)
        if(FETCHCONTENT_FULLY_DISCONNECTED)
            message(FATAL_ERROR
                "pybind11 was not found and FetchContent is fully disconnected. "
                "Install pybind11, set CMAKE_PREFIX_PATH, or configure with network access.")
        endif()
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG        v2.12.0
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(pybind11)
    endif()
endif()

set(SBF_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SBF_LECTDB_INCLUDE_DIR ${SBF_LECT_DATABASE_MODULE_DIR}/include)

add_library(sbf_core STATIC
    src/adaptive_grid_partition.cpp
    src/adaptive_grid_partition_connectivity.cpp
    src/adaptive_grid_partition_geometry.cpp
    src/adaptive_grid_partition_indices.cpp
    src/adaptive_grid_partition_merge.cpp
    src/adaptive_grid_partition_neighbors.cpp
    src/adaptive_grid_partition_overlay.cpp
    src/adaptive_grid_partition_path_query.cpp
    src/adaptive_grid_partition_point_index.cpp
    src/adaptive_grid_partition_query.cpp
    src/adaptive_grid_partition_sparse.cpp
    src/box_graph.cpp
    src/box_graph_edges.cpp
    src/box_graph_search.cpp
    src/box_graph_sequence.cpp
    src/box_graph_topology.cpp
    src/connector.cpp
    src/connector_birrt.cpp
    src/connector_frontier_bridge.cpp
    src/connector_internal.cpp
    src/connector_chain_pave.cpp
    src/find_free_box.cpp
    src/frontwave_grower.cpp
    src/grower.cpp
    src/grower_bootstrap.cpp
    src/grower_component_connect.cpp
    src/grower_components.cpp
    src/grower_failure_cooling.cpp
    src/grower_frontier.cpp
    src/grower_options.cpp
    src/grower_roots.cpp
    src/grower_task_builder.cpp
    src/grower_trace.cpp
    src/grower_workers.cpp
    src/leaf_sweep_grower.cpp
    src/leaf_sweep_grower_cluster.cpp
    src/leaf_sweep_grower_compose.cpp
    src/leaf_sweep_grower_frontier.cpp
    src/merger.cpp
    src/planning_forest_adaptive_build.cpp
    src/planning_forest_adaptive_diagnostics.cpp
    src/planning_forest_adaptive_fast_checkpoint.cpp
    src/planning_forest_adaptive_fixed.cpp
    src/planning_forest_adaptive_cover_utils.cpp
    src/planning_forest_adaptive_merge.cpp
    src/planning_forest_audit.cpp
    src/planning_forest_build.cpp
    src/planning_forest_core.cpp
    src/planning_forest_database.cpp
    src/planning_forest_debug.cpp
    src/planning_forest_diagnostics.cpp
    src/planning_forest_leaf_refine.cpp
    src/planning_forest_dynamic_cache.cpp
    src/planning_forest_dynamic_collision_cache.cpp
    src/planning_forest_dynamic_helpers.cpp
    src/planning_forest_dynamic_refill.cpp
    src/planning_forest_dynamic_segment_fallback.cpp
    src/planning_forest_ffb.cpp
    src/planning_forest_obb.cpp
    src/planning_forest_obb_path_cover.cpp
    src/planning_forest_obb_sampled.cpp
    src/planning_forest_obb_zonotope.cpp
    src/planning_forest_overlay.cpp
    src/planning_forest_overlay_edges.cpp
    src/planning_forest_partition.cpp
    src/planning_forest_qroot_helpers.cpp
    src/planning_forest_qroot_growers.cpp
    src/planning_forest_query.cpp
    src/planning_forest_query_bridge_acceptance_options.cpp
    src/planning_forest_query_bridge_anchor.cpp
    src/planning_forest_query_bridge_attempt_paths.cpp
    src/planning_forest_query_bridge_batch.cpp
    src/planning_forest_query_bridge_batch_diagnostics.cpp
    src/planning_forest_query_bridge_batch_options.cpp
    src/planning_forest_query_bridge_batch_policy.cpp
    src/planning_forest_query_bridge_corridor_options.cpp
    src/planning_forest_query_bridge_corridor_diagnostics.cpp
    src/planning_forest_query_bridge_corridor_graph.cpp
    src/planning_forest_query_bridge_corridor_repair.cpp
    src/planning_forest_query_bridge_corridor_tasks.cpp
    src/planning_forest_query_bridge_direct_corridor.cpp
    src/planning_forest_query_bridge_direct_promotion.cpp
    src/planning_forest_query_bridge_direct_segments.cpp
    src/planning_forest_query_bridge_path_utils.cpp
    src/planning_forest_query_bridge_repair_options.cpp
    src/planning_forest_query_bridge_edges.cpp
    src/planning_forest_query_bridge_endpoint.cpp
    src/planning_forest_query_bridge_endpoint_direct.cpp
    src/planning_forest_query_bridge_hipac.cpp
    src/planning_forest_query_bridge_hipac_utils.cpp
    src/planning_forest_query_bridge_index_options.cpp
    src/planning_forest_query_bridge_pair.cpp
    src/planning_forest_query_bridge_pave.cpp
    src/planning_forest_query_bridge_partition_path_first.cpp
    src/planning_forest_query_bridge_rrt_utils.cpp
    src/planning_forest_query_bridge_task_key.cpp
    src/planning_forest_query_bridge_waypoint_quality.cpp
    src/planning_forest_query_bridge_waypoint.cpp
    src/planning_forest_query_utils.cpp
    src/planning_forest_query_utils_rrt.cpp
    src/planning_forest_query_utils_shortcut.cpp
    src/planning_forest_shortcut.cpp
    src/planning_forest_subtractive.cpp
    src/query.cpp
    src/runtime.cpp
)
add_library(SBF::core ALIAS sbf_core)
set_target_properties(sbf_core PROPERTIES EXPORT_NAME core)

target_include_directories(sbf_core PUBLIC
    $<BUILD_INTERFACE:${SBF_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${SBF_LECTDB_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(sbf_core PUBLIC
    $<BUILD_INTERFACE:${SBF_LECTDB_CORE_TARGET}>
    Threads::Threads
)

if(SBF_LECTDB_ADAPTER_TARGET)
    target_link_libraries(sbf_core PUBLIC ${SBF_LECTDB_ADAPTER_TARGET})
endif()

if(SBF_LECTDB_CACHE_TARGET)
    target_link_libraries(sbf_core PUBLIC ${SBF_LECTDB_CACHE_TARGET})
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(sbf_core PRIVATE -Wall -Wextra -Wpedantic -O3)
endif()

if(SBF_WITH_PYTHON)
    pybind11_add_module(_sbf_cpp python/bindings.cpp)
    target_link_libraries(_sbf_cpp PRIVATE sbf_core ${OMPL_LIBRARIES})
    target_include_directories(_sbf_cpp PRIVATE ${SBF_INCLUDE_DIR} ${OMPL_INCLUDE_DIRS})
    set_target_properties(_sbf_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/sbf
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/sbf
    )

    add_custom_target(sbf_python_package ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/python/sbf
                ${CMAKE_BINARY_DIR}/python/sbf
        DEPENDS _sbf_cpp
        COMMENT "Copying SBF Python package"
    )
endif()

if(SBF_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(SBF_BUILD_EXPERIMENTS)
    add_executable(sbf_toy_bench experiments/toy_bench.cpp)
    target_link_libraries(sbf_toy_bench PRIVATE sbf_core)
    add_executable(tro2026_graph_equivalence_benchmark experiments/tro2026_graph_equivalence_benchmark.cpp)
    target_link_libraries(tro2026_graph_equivalence_benchmark PRIVATE sbf_core)
    if(SBF_BUILD_TESTS)
        add_test(NAME sbf_toy_bench COMMAND sbf_toy_bench)
        add_test(NAME tro2026_graph_equivalence_benchmark COMMAND tro2026_graph_equivalence_benchmark --nx 8 --ny 6 --nz 2)
    endif()
endif()

set(SBF_INSTALL_EXPORT_ARGS)
if(SBF_INSTALL_CMAKE_PACKAGE)
    list(APPEND SBF_INSTALL_EXPORT_ARGS EXPORT SBFTargets)
endif()

install(TARGETS sbf_core
    ${SBF_INSTALL_EXPORT_ARGS}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(SBF_INSTALL_CMAKE_PACKAGE)
    install(EXPORT SBFTargets
        FILE SBFTargets.cmake
        NAMESPACE SBF::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SBF
    )

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/SBFConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/SBFConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SBF
    )
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/SBFConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/SBFConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/SBFConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SBF
    )
endif()
