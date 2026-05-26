add_executable(test_link_interval_envelope test_link_interval_envelope.cpp)
target_link_libraries(test_link_interval_envelope PRIVATE link_interval_envelope_core)
target_compile_definitions(test_link_interval_envelope PRIVATE
    LIE_EXAMPLE_DATA_DIR="${CMAKE_SOURCE_DIR}/examples/data"
)
add_test(NAME test_link_interval_envelope COMMAND test_link_interval_envelope)
set_tests_properties(test_link_interval_envelope PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

if(LIE_WITH_PYTHON)
    add_test(
        NAME test_python_api
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_python_api.py
    )
    set_tests_properties(test_python_api PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/python:${CMAKE_SOURCE_DIR}/python"
    )
endif()

