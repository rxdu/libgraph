# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Google tests
find_package(GTest QUIET)
if(GTEST_FOUND)
    message(STATUS "Found Google Test. Will build unit tests using gtest.")
    add_subdirectory(gtests)
else(GTEST_FOUND)
    message(STATUS "Google Test not found. Disabled unit tests using gtest.")
endif(GTEST_FOUND)

# Other tests
#add_executable(test_graph graph_tests.cpp)
add_executable(graph_types others/graph_types.cpp)
add_executable(test_heuristic others/test_heuristic.cpp)
add_executable(test_iterator others/test_iterator.cpp)
#add_executable(test_performance performance_test.cpp)
#add_executable(remove_vertex remove_vertex.cpp)