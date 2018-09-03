# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
add_executable(test_graph_creation test_graph_creation.cpp)
target_link_libraries(test_graph_creation graph2)