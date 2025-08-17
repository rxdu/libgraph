# Add demo executables
add_executable(simple_graph_demo simple_graph_demo.cpp)
target_link_libraries(simple_graph_demo graph)

add_executable(graph_type_demo graph_type_demo.cpp)
target_link_libraries(graph_type_demo graph)

add_executable(inc_search_demo inc_search_demo.cpp)
target_link_libraries(inc_search_demo graph)

# Flexible search examples (commented out due to iostream dependency issue)
# add_executable(flexible_search_examples flexible_search_examples.cpp)
# target_link_libraries(flexible_search_examples graph)
