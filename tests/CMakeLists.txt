if(OMPL_IN_ROS)
    option(OMPL_BUILD_TESTS "Build OMPL tests" ON)
else(OMPL_IN_ROS)
    find_package(GTest)
    option(OMPL_BUILD_TESTS "Build OMPL tests" ${GTEST_FOUND})
endif(OMPL_IN_ROS)

if(OMPL_BUILD_TESTS)

    # include location of common headers used in testing
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    # configure location of resources
    file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
    configure_file("${TEST_RESOURCES_DIR}/config.h.in" "${TEST_RESOURCES_DIR}/config.h")


    # define macros for adding tests
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

    # add actual tests for ompl
    add_subdirectory(ompl)

endif(OMPL_BUILD_TESTS)
