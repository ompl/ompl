option(ROS "Build against ROS" FALSE)

if(ROS)
	rosbuild_add_boost_directories()
else(ROS)
	cmake_minimum_required(VERSION 2.6)
	project(ompl CXX)
	set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
	set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

	set(CMAKE_PREFIX_PATH 
		${CMAKE_CURRENT_SOURCE_DIR}/external/sandbox
		${CMAKE_PREFIX_PATH})
	
	include(FindPackageHandleStandardArgs)
	find_package(Boost COMPONENTS date_time thread filesystem system REQUIRED)
	include_directories("${Boost_INCLUDE_DIR}")
	add_definitions(-W -Wall -Wextra -Wno-missing-field-initializers)

	enable_testing()
endif(ROS)

include_directories(code)
add_subdirectory(code)
