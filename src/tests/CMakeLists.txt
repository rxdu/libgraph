# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
## examples
#add_executable(test_graph graph_tests.cpp)

add_executable(graph_types graph_types.cpp)

add_executable(test_heuristic test_heuristic.cpp)

#add_executable(test_performance performance_test.cpp)

#add_executable(remove_vertex remove_vertex.cpp)