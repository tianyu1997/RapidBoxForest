set(LECT_DATABASE_TEST_TMPDIR "${CMAKE_CURRENT_BINARY_DIR}/tmp")
file(MAKE_DIRECTORY "${LECT_DATABASE_TEST_TMPDIR}")

add_executable(test_lect_database_cpp
    test_lect_database.cpp
)
target_link_libraries(test_lect_database_cpp PRIVATE lect_database_core)
add_test(NAME test_lect_database_cpp COMMAND test_lect_database_cpp)
set_tests_properties(test_lect_database_cpp PROPERTIES
    ENVIRONMENT "TMPDIR=${LECT_DATABASE_TEST_TMPDIR}"
)

add_executable(test_lect_database_header_surface_cpp
    header_surface/main.cpp
    header_surface/lect_database_header.cpp
)
target_link_libraries(test_lect_database_header_surface_cpp PRIVATE lect_database_core)
add_test(NAME test_lect_database_header_surface_cpp COMMAND test_lect_database_header_surface_cpp)

add_executable(test_sbf_adapter_cpp
    test_sbf_adapter.cpp
)
target_link_libraries(test_sbf_adapter_cpp PRIVATE lect_database_sbf_adapter)
add_test(NAME test_sbf_adapter_cpp COMMAND test_sbf_adapter_cpp)
set_tests_properties(test_sbf_adapter_cpp PROPERTIES
    ENVIRONMENT "TMPDIR=${LECT_DATABASE_TEST_TMPDIR}"
)

add_executable(test_online_cache_cpp
    test_online_cache.cpp
)
target_link_libraries(test_online_cache_cpp PRIVATE lect_database_online_cache)
add_test(NAME test_online_cache_cpp COMMAND test_online_cache_cpp)
set_tests_properties(test_online_cache_cpp PROPERTIES
    ENVIRONMENT "TMPDIR=${LECT_DATABASE_TEST_TMPDIR}"
)
