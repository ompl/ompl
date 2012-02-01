if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-W -Wall -Wextra -Wno-missing-field-initializers -Wno-unused)
endif(CMAKE_COMPILER_IS_GNUCXX)

string(REGEX MATCH ".*icpc" IS_ICPC ${CMAKE_CXX_COMPILER})
if(IS_ICPC)
    set(CMAKE_AR "xiar" CACHE STRING "Intel archiver" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-fast -DNDEBUG -pthread"
    CACHE STRING "Flags used by the C++ compiler during release builds." FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -pthread" CACHE STRING
    "Flags used by the C++ compiler during debug builds." FORCE)
endif(IS_ICPC)

string(REGEX MATCH ".*xlC" IS_XLC ${CMAKE_CXX_COMPILER})
if(IS_XLC)
    add_definitions(-qpic -q64 -qmaxmem=-1)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -q64")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -q64")
endif(IS_XLC)

if(CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC)
    add_definitions(-fPIC)
endif(CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC)

# Set rpath http://www.paraview.org/Wiki/CMake_RPATH_handling
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# no prefix needed for python modules
set(CMAKE_SHARED_MODULE_PREFIX "")
