macro(add_ompl_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ompl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_ompl_test)

macro(add_ompl_python_test test_file)
  get_filename_component(test_name "${test_file}" NAME)
  add_test(${test_name} "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}" "-v")
endmacro(add_ompl_python_test)

if(UNIX)
    option(OMPL_VERSIONED_INSTALL "Append version suffix to binaries, libraries, and include dir." ON)
else()
    option(OMPL_VERSIONED_INSTALL "Append version suffix to binaries, libraries, and include dir." OFF)
endif()

if (OMPL_VERSIONED_INSTALL)
    set(OMPL_INSTALL_SUFFIX "-${OMPL_MAJOR_VERSION}.${OMPL_MINOR_VERSION}")
    macro(ompl_install_symlink path file)
        get_filename_component(base ${file} NAME_WE)
        get_filename_component(ext ${file} EXT)
        install(CODE "execute_process(COMMAND mkdir -p \"${CMAKE_INSTALL_PREFIX}/${path}\"
            COMMAND ln -sf
            \"${CMAKE_INSTALL_PREFIX}/${path}/${base}${OMPL_INSTALL_SUFFIX}${ext}\"
            \"${CMAKE_INSTALL_PREFIX}/${path}/${file}\")")
    endmacro(ompl_install_symlink)
else()
    set(OMPL_INSTALL_SUFFIX "")
    macro(ompl_install_symlink path file)
        # do nothing
    endmacro(ompl_install_symlink)
endif()
