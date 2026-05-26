include(FetchContent)

# Keep third-party FetchContent projects from registering their own CTest suites.
# The package tests are controlled by LIE_BUILD_TESTS in the top-level project.
set(BUILD_TESTING OFF CACHE BOOL "Disable dependency test suites" FORCE)

option(LIE_FORCE_FETCH_DEPS "Fetch third-party dependencies even if find_package can see installed packages" OFF)
option(LIE_FORCE_FETCH_EIGEN "Fetch Eigen even if find_package can see an installed package" ${LIE_FORCE_FETCH_DEPS})
option(LIE_FORCE_FETCH_NLOHMANN_JSON "Fetch nlohmann_json even if find_package can see an installed package" ${LIE_FORCE_FETCH_DEPS})

if(NOT LIE_FORCE_FETCH_EIGEN)
    find_package(Eigen3 3.4 QUIET)
endif()
if(NOT Eigen3_FOUND)
    FetchContent_Declare(
        eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
        GIT_SHALLOW    TRUE
        GIT_CONFIG     core.longpaths=true
    )
    set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen3)
endif()

if(NOT LIE_FORCE_FETCH_NLOHMANN_JSON)
    find_package(nlohmann_json 3.11 QUIET)
endif()
if(NOT nlohmann_json_FOUND)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE
        GIT_CONFIG     core.longpaths=true
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()

if(LIE_WITH_PYTHON)
    set(PYBIND11_FINDPYTHON ON CACHE BOOL "Use CMake FindPython for pybind11" FORCE)
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

    find_package(pybind11 2.12 QUIET)
    if(NOT pybind11_FOUND)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG        v2.12.0
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(pybind11)
    endif()
endif()
