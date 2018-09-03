include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${GTEST_INCLUDE_DIRS})

# Add executables
set(TESTCASES_SRC
    src/graph_type_test.cpp
    src/graph_mod_test.cpp
    # src/graph_iter_test.cpp
    # src/graph_search_test.cpp
    # src/graph_search_inc_test.cpp
)
add_executable(gtest_all src/start_tests.cpp ${TESTCASES_SRC})
target_link_libraries(gtest_all gtest graph2)