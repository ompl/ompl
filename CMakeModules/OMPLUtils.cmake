macro(add_ompl_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ompl
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SERIALIZATION_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  add_test(NAME ${test_name} COMMAND $<TARGET_FILE:${test_name}>)
endmacro(add_ompl_test)

macro(add_ompl_python_test test_file)
  get_filename_component(test_name "${test_file}" NAME)
  add_test(${test_name} "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}" "-v")
endmacro(add_ompl_python_test)

function(target_link_flags)
    set(_link_flags "")
    foreach(_target ${ARGV})
        get_target_property(_link_libs ${_target} LINK_LIBRARIES)
        foreach(_lib ${_link_libs})
            get_filename_component(_basename ${_lib} NAME_WE)
            get_filename_component(_ext ${_lib} EXT)
            if(_ext STREQUAL ${CMAKE_SHARED_LIBRARY_SUFFIX})
                string(REPLACE ${CMAKE_SHARED_LIBRARY_PREFIX} "" _libname ${_basename})
                list(APPEND _link_flags "-l${_libname}")
            endif()
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES _link_flags)
    foreach(_flag ${_link_flags})
        set(_link_flags_str "${_link_flags_str} ${_flag}")
    endforeach()
    string(STRIP "${_link_flags_str}" _link_flags_str)
    set(${ARGV0}_LINK_FLAGS "${_link_flags_str}" PARENT_SCOPE)
endfunction()

option(OMPL_VERSIONED_INSTALL "Append version suffix to binaries, libraries, and include dir." OFF)
if (OMPL_VERSIONED_INSTALL)
    set(OMPL_INSTALL_SUFFIX "-${OMPL_MAJOR_VERSION}.${OMPL_MINOR_VERSION}")
else()
    set(OMPL_INSTALL_SUFFIX "")
endif()
