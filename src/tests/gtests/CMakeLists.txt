include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${GTEST_INCLUDE_DIRS})

# Add executables
set(TESTCASES_SRC
    graph_type_test.cpp
    graph_iter_test.cpp
)
add_executable(gtest_all start_tests.cpp ${TESTCASES_SRC})
target_link_libraries(gtest_all gtest)