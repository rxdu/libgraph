# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
add_executable(graph_types graph_types.cpp)
add_executable(test_heuristic test_heuristic.cpp)
add_executable(test_iterator test_iterator.cpp)
add_executable(test_ref_graph test_ref_graph.cpp)
add_executable(test_ref_graph2 test_ref_graph2.cpp)
add_executable(test_pt_graph test_pt_graph.cpp)