find_package(GTest)
find_package(Threads)
option(OMPL_BUILD_TESTS "Build OMPL tests" ${GTEST_FOUND})

# define macros for adding tests
if(GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
endif()

macro(add_ompl_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ompl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY} ${GTEST_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_ompl_test)

macro(add_ompl_python_test test_file)
  get_filename_component(test_name "${test_file}" NAME)
  add_test(${test_name} "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}" "-v")
endmacro(add_ompl_python_test)
