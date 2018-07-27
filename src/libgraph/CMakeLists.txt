# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(graph INTERFACE)
target_sources(graph INTERFACE 
	${CMAKE_CURRENT_SOURCE_DIR}/graph/graph.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/vertex.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/edge.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/graph_impl.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/vertex_impl.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/edge_impl.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/helper_func.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/details/priority_queue.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/algorithms/astar.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/graph/algorithms/dijkstra.hpp
	)
target_compile_definitions(graph INTERFACE -DUSE_UNORDERED_MAP -DMINIMAL_PRINTOUT)
target_include_directories(graph INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)