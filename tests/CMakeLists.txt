# Google tests
message(STATUS "Build unit tests with Google Test.")
add_subdirectory(googletest)

# reference: https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

# add unit tests
add_executable(utests 
    unit_test/priority_queue_test.cpp
    unit_test/pq_with_graph_test.cpp
    unit_test/graph_bigfive_test.cpp
    unit_test/graph_type_test.cpp
    unit_test/graph_mod_test.cpp
    unit_test/graph_iter_test.cpp
    unit_test/graph_search_test.cpp
    unit_test/graph_search_inc_test.cpp
    unit_test/tree_bigfive_test.cpp
    unit_test/tree_mod_test.cpp)
target_link_libraries(utests PRIVATE gtest gmock gtest_main graph)
# get_target_property(PRIVATE_HEADERS graph INCLUDE_DIRECTORIES)
target_include_directories(utests PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(utests)
add_test(NAME gtest_all COMMAND utests)

# additional tests 
# add_subdirectory(devel_test)
