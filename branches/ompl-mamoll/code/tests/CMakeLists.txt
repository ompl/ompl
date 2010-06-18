if(ROS)  
	option(BUILD_TESTS "Build ompl tests" TRUE) 
else(ROS)
	find_package(GTest)
	option(BUILD_TESTS "Build ompl tests" ${GTEST_FOUND})
endif(ROS)


if(BUILD_TESTS)
	file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
	configure_file("${TEST_RESOURCES_DIR}/config.h.in" "${TEST_RESOURCES_DIR}/config.h")
	
	if(ROS)

		macro(add_ompl_test test_name)
			rosbuild_add_gtest(${ARGV})
			target_link_libraries(${test_name} ompl)
			rosbuild_link_boost(test_2dmap_kinematic filesystem system thread date_time)
		endmacro(add_ompl_test)

	else(ROS)
		include_directories(${GTEST_INCLUDE_DIRS})

		macro(add_ompl_test test_name)
			add_executable(${ARGV})
			target_link_libraries(${test_name} 
				ompl
				${Boost_FILESYSTEM_LIBRARY}
				${Boost_SYSTEM_LIBRARY}
				${Boost_THREAD_LIBRARY}
			${Boost_DATE_TIME_LIBRARY} ${GTEST_LIBRARIES})
			add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
		endmacro(add_ompl_test)

	endif(ROS)

	# Test kinematic motion planners on a 2D map
	add_ompl_test(test_2dmap_kinematic kinematic/2dmap/2dmap.cpp)

	# Test dynamic motion planners on a 2D map
	add_ompl_test(test_2dmap_dynamic dynamic/2dmap/2dmap.cpp)

	# Test datastructures implementation
	add_ompl_test(test_heap datastructures/heap/heap.cpp)
	add_ompl_test(test_grid datastructures/grid/grid.cpp)
	add_ompl_test(test_gridb datastructures/grid/gridb.cpp)

	# Test utilities
	add_ompl_test(test_random util/random/random.cpp)

	# Test compilation
	add_ompl_test(test_compile compile/compile.cpp)

endif(BUILD_TESTS)
