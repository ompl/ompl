include(FindPackageHandleStandardArgs)

if(NOT CASTXML)
    find_program(CASTXML NAMES castxml)
endif()

if (CASTXML)
    set(CASTXMLCFLAGS "-std=c++17 -fsized-deallocation $ENV{CASTXMLCFLAGS}")

    set(CASTXMLCOMPILER "${CMAKE_CXX_COMPILER_ID}")
    set(CASTXMLCOMPILER_PATH "${CMAKE_CXX_COMPILER}")

    set(CASTXMLCONFIG "[xml_generator]
xml_generator=castxml
xml_generator_path=${CASTXML}
compiler=${CASTXMLCOMPILER}
compiler_path=${CASTXMLCOMPILER_PATH}
")

    set(_candidate_include_path
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/ompl/src"
        "${CMAKE_BINARY_DIR}/src"
        "${CMAKE_BINARY_DIR}/ompl/src"
        "${PYTHON_INCLUDE_DIRS}"
        "${Boost_INCLUDE_DIR}"
        "${ASSIMP_INCLUDE_DIRS}"
        "${EIGEN3_INCLUDE_DIR}"
        "${CMAKE_SOURCE_DIR}/py-bindings"
        "${CMAKE_SOURCE_DIR}/ompl/py-bindings")
    if(MINGW)
        execute_process(COMMAND "${CMAKE_CXX_COMPILER}" "-dumpversion"
            OUTPUT_VARIABLE _version OUTPUT_STRIP_TRAILING_WHITESPACE)
        get_filename_component(_path "${CMAKE_CXX_COMPILER}" DIRECTORY)
        get_filename_component(_path "${_path}" DIRECTORY)
        list(APPEND _candidate_include_path
            "${_path}/include"
            "${_path}/lib/gcc/mingw32/${_version}/include"
            "${_path}/lib/gcc/mingw32/${_version}/include/c++"
            "${_path}/lib/gcc/mingw32/${_version}/include/c++/mingw32")
    endif()

    if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "arm64")
      # Some shell magic to extract out the default includes that are NOT included in castxml by default.
      # Note there may be some issue with the `cut` if a path has spaces, hopefully not since these are system includes
      execute_process(COMMAND bash "-c" "${CASTXMLCOMPILER} -E -x c++ - -v < /dev/null 2>&1 | awk '/#include <...> search starts here:/{flag=1;next}/End of search list./{flag=0}flag' - | cut -f 2 -d' ' - | paste -sd\; -"
        OUTPUT_VARIABLE _system_include_paths OUTPUT_STRIP_TRAILING_WHITESPACE)

      # Append those default includes
      list(APPEND _candidate_include_path ${_system_include_paths})

      # Some issues with ARM Neon intrinsics breaking in the castxml parse step, ignore in this part
      set(CASTXMLCFLAGS "${CASTXMLCFLAGS} -DEIGEN_DONT_VECTORIZE")
    endif()

    list(REMOVE_DUPLICATES _candidate_include_path)
    set(CASTXMLINCLUDEPATH ".")
    foreach(dir ${_candidate_include_path})
        if(EXISTS ${dir})
            set(CASTXMLINCLUDEPATH "${CASTXMLINCLUDEPATH};${dir}")
        endif()
    endforeach()
    set(CASTXMLCONFIG "${CASTXMLCONFIG}include_paths=${CASTXMLINCLUDEPATH}\n")
    if(CASTXMLCFLAGS)
        set(CASTXMLCONFIG "${CASTXMLCONFIG}cflags=${CASTXMLCFLAGS}\n")
    endif()
    set(CASTXMLCONFIGPATH "${PROJECT_BINARY_DIR}/castxml.cfg")
    file(WRITE "${CASTXMLCONFIGPATH}" "${CASTXMLCONFIG}")
    set(CASTXMLCONFIGPATH "${CASTXMLCONFIGPATH}" PARENT_SCOPE)
endif()

find_package_handle_standard_args(castxml DEFAULT_MSG CASTXML)
