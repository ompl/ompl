include(FindPackageHandleStandardArgs)

if(NOT XMLGENERATORPATH)
    find_program(XMLGENERATORPATH NAMES castxml)
    if(XMLGENERATORPATH)
        set(XMLGENERATOR "castxml" CACHE STRING "Type of XML generator used by pygccxml")
    else()
        find_program(XMLGENERATORPATH NAMES gccxml
            PATHS "${PROJECT_BINARY_DIR}/pyplusplus/bin"
            [HKEY_CURRENT_USER\\Software\\Kitware\\GCC_XML;loc]
            "$ENV{ProgramFiles}/GCC_XML"
            "C:/Program Files/GCC_XML")
        if (XMLGENERATORPATH)
            set(XMLGENERATOR "gccxml" CACHE STRING "Type of XML generator used by pygccxml")
        endif()
    endif()
endif()

if (XMLGENERATORPATH)
    # Gccxml mistakenly thinks that OS X is a 32-bit architecture.
    if(XMLGENERATOR STREQUAL "gccxml" AND APPLE)
        set(XMLCFLAGS "-m64")
    endif()

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(XMLCOMPILER "g++")
        # Trick gccxml to ignore some compiler intrinsics that are used in Boost.Atomic
        # in Boost 1.55.
        if(Boost_VERSION VERSION_GREATER "105400")
            set(XMLCFLAGS "${XMLCFLAGS} -DBOOST_INTEL_CXX_VERSION")
        endif()
    else()
        if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            set(XMLCOMPILER "clang++")
        else()
            if (MSVC)
                set(XMLCOMPILER "msvc8")
            endif()
        endif()
    endif()

    set(XMLCONFIG "[gccxml]\nxml_generator=${XMLGENERATOR}\nxml_generator_path=${XMLGENERATORPATH}\n")
    if (XMLCOMPILER AND NOT (APPLE AND XMLGENERATOR STREQUAL "gccxml"))
        set(XMLCONFIG "${XMLCONFIG}compiler=${XMLCOMPILER}\ncompiler_path=${CMAKE_CXX_COMPILER}\n")
    endif()

    set(_candidate_include_path
        "${OMPL_INCLUDE_DIR}"
        "${OMPLAPP_INCLUDE_DIR}"
        "${PYTHON_INCLUDE_DIRS}"
        "${Boost_INCLUDE_DIR}"
        "${ASSIMP_INCLUDE_DIRS}"
        "${ODEINT_INCLUDE_DIR}"
        "${EIGEN3_INCLUDE_DIR}")
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
    set(XMLINCLUDEPATH ".")
    foreach(dir ${_candidate_include_path})
        if(EXISTS ${dir})
            set(XMLINCLUDEPATH "${XMLINCLUDEPATH};${dir}")
        endif()
    endforeach()
    set(XMLCONFIG "${XMLCONFIG}include_paths=${XMLINCLUDEPATH}\n")
    if(XMLCFLAGS)
        set(XMLCONFIG "${XMLCONFIG}cflags=${XMLCFLAGS}\n")
    endif()
    set(XMLCONFIGPATH "${PROJECT_BINARY_DIR}/xmlgenerator.cfg")
    file(WRITE "${XMLCONFIGPATH}" "${XMLCONFIG}")
    set(XMLCONFIGPATH "${XMLCONFIGPATH}" PARENT_SCOPE)
endif()

find_package_handle_standard_args(xmlgenerator DEFAULT_MSG XMLGENERATORPATH)

