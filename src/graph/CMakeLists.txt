# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(graph INTERFACE)
target_sources(graph INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/graph/tree.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/graph/graph.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/graph/details/default_indexer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/graph/details/priority_queue.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/graph/details/edge_impl.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/graph/details/vertex_impl.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/graph/details/graph_impl.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/graph/details/tree_impl.hpp
)
target_compile_definitions(graph INTERFACE -DMINIMAL_PRINTOUT)
target_include_directories(graph INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
