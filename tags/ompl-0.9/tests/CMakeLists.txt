if(OMPL_IN_ROS)  
	option(OMPL_BUILD_TESTS "Build OMPL tests" ON) 
else(OMPL_IN_ROS)
	find_package(GTest)
	option(OMPL_BUILD_TESTS "Build OMPL tests" ${GTEST_FOUND})
endif(OMPL_IN_ROS)

if(OMPL_BUILD_TESTS)

	file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
	configure_file("${TEST_RESOURCES_DIR}/config.h.in" "${TEST_RESOURCES_DIR}/config.h")
	
	if(OMPL_IN_ROS)

		macro(add_ompl_test test_name)
			rosbuild_add_gtest(${ARGV})
			target_link_libraries(${test_name} ompl)
			rosbuild_link_boost(${test_name} filesystem system thread date_time)
		endmacro(add_ompl_test)

	        macro(add_ompl_python_test test_file)
#  Disable Python tests for ROS
#                        rosbuild_add_pyunit(${test_file})
	        endmacro(add_ompl_python_test)

	else(OMPL_IN_ROS)
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

	        macro(add_ompl_python_test test_file)
		        get_filename_component(test_name "${test_file}" NAME)
		        add_test(${test_name} "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}" "-v")
	        endmacro(add_ompl_python_test)

	endif(OMPL_IN_ROS)
	
	# Test datastructures implementation
	add_ompl_test(test_heap datastructures/heap/heap.cpp)
	add_ompl_test(test_grid datastructures/grid/grid.cpp)
	add_ompl_test(test_gridb datastructures/grid/gridb.cpp)

	# Test utilities
	add_ompl_test(test_random util/random/random.cpp)
	add_ompl_test(test_memory util/memory/memory.cpp)

	# Test base code
	add_ompl_test(test_state_operations base/state_operations.cpp)
	add_ompl_test(test_manifolds base/manifolds.cpp)

	# Test kinematic motion planners on a 2D map
	add_ompl_test(test_2dmap_geometric geometric/2dmap/2dmap.cpp)
	add_ompl_test(test_2dmap_geometric_simple geometric/2dmap/2dmap_simple.cpp)
	add_ompl_test(test_2dmap_ik geometric/2dmap/2dmap_ik.cpp)

	# Test planning with controls on a 2D map
	add_ompl_test(test_2dmap_control control/2dmap/2dmap.cpp)

	# Python unit tests
	if(OMPL_BUILD_PYTESTS)
		add_ompl_python_test(util/test_util.py)
		add_ompl_python_test(base/test_base.py)
		add_ompl_python_test(geometric/test_geometric.py)
		add_ompl_python_test(geometric/test_geometric_compoundstate.py)
		add_ompl_python_test(control/test_control.py)
	endif()
	
endif(OMPL_BUILD_TESTS)
