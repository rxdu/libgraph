# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(graph INTERFACE)
target_compile_definitions(graph INTERFACE -DMINIMAL_PRINTOUT)
target_include_directories(graph INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

add_subdirectory(demo)