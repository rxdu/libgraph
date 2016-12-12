# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
## examples
add_executable(test_graph graph_tests.cpp)

add_executable(test_performance performance_test.cpp)

add_executable(remove_vertex remove_vertex.cpp)