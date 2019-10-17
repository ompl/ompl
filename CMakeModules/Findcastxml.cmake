include(FindPackageHandleStandardArgs)

if(NOT CASTXML)
    find_program(CASTXML NAMES castxml)
endif()

if (CASTXML)
    set(CASTXMLCFLAGS "-std=c++14 $ENV{CASTXMLCFLAGS}")

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CASTXMLCOMPILER "g++")
    else()
        if(CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
            set(CASTXMLCOMPILER "clang++")
        else()
            if (MSVC)
                set(CASTXMLCOMPILER "msvc8")
            endif()
        endif()
    endif()

    # workaround for problem between Xcode and castxml on Mojave
    if (APPLE AND CMAKE_CXX_COMPILER MATCHES "/Applications/Xcode.app/Contents/Developer/Toolchains/.*")
        set(CASTXMLCOMPILER_PATH "/usr/bin/clang++")
    else()
        set(CASTXMLCOMPILER_PATH "${CMAKE_CXX_COMPILER}")
    endif()

    set(CASTXMLCONFIG "[xml_generator]
xml_generator=castxml
xml_generator_path=${CASTXML}
compiler=${CASTXMLCOMPILER}
compiler_path=${CASTXMLCOMPILER_PATH}
")

    set(_candidate_include_path
        "${OMPL_INCLUDE_DIRS}"
        "${OMPLAPP_INCLUDE_DIRS}"
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
