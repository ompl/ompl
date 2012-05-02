find_package(Threads)

macro(add_ompl_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ompl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${Boost_TEST_EXEC_MONITOR_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT})
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_ompl_test)

macro(add_ompl_python_test test_file)
  get_filename_component(test_name "${test_file}" NAME)
  add_test(${test_name} "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}" "-v")
endmacro(add_ompl_python_test)
