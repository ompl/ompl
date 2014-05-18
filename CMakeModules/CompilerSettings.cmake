if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-W -Wall -Wextra #-Wconversion
                    -Wcast-qual -Wwrite-strings -Wunreachable-code -Wpointer-arith
                    -Winit-self -Wredundant-decls
                    -Wno-unused-parameter -Wno-unused-function)
    # prepend optimizion flag (in case the default setting doesn't include one)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
endif(CMAKE_COMPILER_IS_GNUCXX)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_definitions(-W -Wall -Wextra -Wno-missing-field-initializers -Wno-unused -Wno-unused-parameter -Wno-delete-non-virtual-dtor -Wno-overloaded-virtual -Wno-unknown-pragmas -Qunused-arguments)
    # prepend optimizion flag (in case the default setting doesn't include one)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

if(MSVC OR MSVC90 OR MSVC10)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP /W1 /D_ITERATOR_DEBUG_LEVEL=0")
endif(MSVC OR MSVC90 OR MSVC10)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    set(IS_ICPC 1)
else()
    set(IS_ICPC 0)
endif()
if(IS_ICPC)
    add_definitions(-wd191 -wd411 -wd654 -wd1125 -wd1292 -wd1565 -wd1628 -wd2196)
    set(CMAKE_AR "xiar" CACHE STRING "Intel archiver" FORCE)
    set(CMAKE_CXX_FLAGS "-pthread" CACHE STRING "Default compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG"
    CACHE STRING "Flags used by the C++ compiler during release builds." FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING
    "Flags used by the C++ compiler during debug builds." FORCE)
    set(CMAKE_LINKER "xild" CACHE STRING "Intel linker" FORCE)
endif(IS_ICPC)

if(CMAKE_CXX_COMPILER_ID STREQUAL "XL")
    set(IS_XLC 1)
else()
    set(IS_XLC 0)
endif()
if(IS_XLC)
    add_definitions(-qpic -q64 -qmaxmem=-1)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -q64")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -q64")
endif(IS_XLC)

if((CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC) AND NOT MINGW)
    add_definitions(-fPIC)
endif((CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC) AND NOT MINGW)

# Set rpath http://www.paraview.org/Wiki/CMake_RPATH_handling
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# no prefix needed for python modules
set(CMAKE_SHARED_MODULE_PREFIX "")
