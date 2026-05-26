cmake_minimum_required(VERSION 3.20)
project(LinkIntervalEnvelope VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(LIE_WITH_PYTHON "Build Python extension" ON)
option(LIE_BUILD_TESTS "Build C++ smoke tests" ON)

include(cmake/FetchDeps.cmake)
find_package(Threads REQUIRED)

set(LIE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(link_interval_envelope_core STATIC
    src/batch.cpp
    src/incremental_context.cpp
    src/core/robot.cpp
    src/core/interval_math.cpp
    src/core/joint_symmetry.cpp
    src/core/fk_state.cpp
    src/core/aa_fk.cpp
    src/envelope/ifk_aa_source.cpp
    src/envelope/crit_source.cpp
    src/envelope/mc_source.cpp
    src/envelope/analytical_source.cpp
    src/envelope/gcpc_source.cpp
    src/envelope/endpoint_source.cpp
    src/envelope/envelope_type.cpp
    src/envelope/link_iaabb.cpp
    src/envelope/link_grid.cpp
    src/envelope/dh_enumerate.cpp
    src/voxel/hull_rasteriser.cpp
)
add_library(link_interval_envelope::core ALIAS link_interval_envelope_core)

target_include_directories(link_interval_envelope_core PUBLIC
    $<BUILD_INTERFACE:${LIE_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(link_interval_envelope_core PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(link_interval_envelope_core PRIVATE -Wall -Wextra -Wpedantic -O3)
endif()

add_executable(lie_microbench examples/microbench.cpp)
target_link_libraries(lie_microbench PRIVATE link_interval_envelope_core)
target_compile_definitions(lie_microbench PRIVATE
    LIE_EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples/data"
)

if(LIE_WITH_PYTHON)
    if(NOT TARGET pybind11::module)
        find_package(pybind11 REQUIRED)
    endif()

    pybind11_add_module(_link_interval_envelope_cpp python/bindings.cpp)
    target_link_libraries(_link_interval_envelope_cpp PRIVATE link_interval_envelope_core)
    target_include_directories(_link_interval_envelope_cpp PRIVATE ${LIE_INCLUDE_DIR})
    set_target_properties(_link_interval_envelope_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/link_interval_envelope
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/link_interval_envelope
    )

    add_custom_target(link_interval_envelope_python_package ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/python/link_interval_envelope
                ${CMAKE_BINARY_DIR}/python/link_interval_envelope
        DEPENDS _link_interval_envelope_cpp
        COMMENT "Copying link_interval_envelope Python package"
    )
endif()

if(LIE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS link_interval_envelope_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
