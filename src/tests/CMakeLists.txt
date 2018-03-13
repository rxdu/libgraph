# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Google tests
if(BUILD_TEST)
    message(STATUS "Build unit tests with Google Test.")
    add_subdirectory(googletest-release-1.8.0)
    add_subdirectory(gtests)
endif(BUILD_TEST)

# Other tests
add_subdirectory(feature_test)