# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
# add_executable(graph_types graph_types.cpp)
# target_link_libraries(graph_types graph)

# add_executable(test_heuristic test_heuristic.cpp)
# target_link_libraries(test_heuristic graph)

# add_executable(test_iterator test_iterator.cpp)
# target_link_libraries(test_iterator graph)

# add_executable(test_pt_graph test_pt_graph.cpp)
# target_link_libraries(test_pt_graph graph)

add_executable(test_default_indexer test_default_indexer.cpp)
target_link_libraries(test_default_indexer graph)

add_executable(test_remove_vertex test_remove_vertex.cpp)
target_link_libraries(test_remove_vertex graph)

add_executable(test_graph_creation test_graph_creation.cpp)
target_link_libraries(test_graph_creation graph)

add_executable(test_tree test_tree.cpp)
target_link_libraries(test_tree graph)

add_executable(test_tree_creation test_tree_creation.cpp)
target_link_libraries(test_tree_creation graph)

# add_executable(test_dynamic_pq test_dynamic_pq.cpp)
# target_link_libraries(test_dynamic_pq graph)

add_executable(test_id test_id.cpp)
# target_link_libraries(test_id graph)

add_executable(test_queue test_queue.cpp)
target_link_libraries(test_queue graph)

add_executable(test_dijkstra test_dijkstra.cpp)
target_link_libraries(test_dijkstra graph)

add_executable(test_astar test_astar.cpp)
target_link_libraries(test_astar graph)

add_executable(test_search_framework test_search_framework.cpp)
target_link_libraries(test_search_framework graph)

add_executable(test_dfs test_dfs.cpp)
target_link_libraries(test_dfs graph)
