# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(graph2 INTERFACE)
# target_sources(graph2 INTERFACE 
# 	${CMAKE_CURRENT_SOURCE_DIR}/include/graph/graph.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/vertex.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/edge.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/graph_impl.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/vertex_impl.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/edge_impl.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/helper_func.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/priority_queue.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/algorithms/astar.hpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/graph/algorithms/dijkstra.hpp
# )
target_compile_definitions(graph2 INTERFACE -DUSE_UNORDERED_MAP -DMINIMAL_PRINTOUT)
target_include_directories(graph2 INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

add_subdirectory(tests)