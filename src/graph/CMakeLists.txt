# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(GRAPH_LIB_SRC
	graph.hpp
	vertex.hpp
	edge.hpp
	details/graph_impl.hpp
	detials/vertex_impl.hpp
	details/edge_impl.hpp
	detials/helper_func.hpp
	details/priority_queue.hpp
	algorithms/astar.hpp
)
add_library(graph INTERFACE)
target_sources(graph INTERFACE ${GRAPH_LIB_SRC})
target_include_directories(graph INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)