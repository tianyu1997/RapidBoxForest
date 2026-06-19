add_executable(test_sbf test_sbf.cpp)
target_link_libraries(test_sbf PRIVATE sbf_core)
add_test(NAME test_sbf COMMAND test_sbf)

add_executable(connector_birrt_benchmark connector_birrt_benchmark.cpp)
target_link_libraries(connector_birrt_benchmark PRIVATE sbf_core)

add_executable(sbf_graph_equivalence_benchmark graph_equivalence_benchmark.cpp)
target_link_libraries(sbf_graph_equivalence_benchmark PRIVATE sbf_core)
add_test(NAME sbf_graph_equivalence_benchmark COMMAND sbf_graph_equivalence_benchmark --nx 8 --ny 6 --nz 2)

if(SBF_WITH_PYTHON)
    add_test(NAME test_sbf_python
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_python_api.py)
    set_tests_properties(test_sbf_python PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/python")
endif()
