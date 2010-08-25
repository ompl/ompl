if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-W -Wall -Wextra -Wno-missing-field-initializers -Wno-unused)
endif(CMAKE_COMPILER_IS_GNUCXX)

string(REGEX MATCH ".*icpc" IS_ICPC ${CMAKE_CXX_COMPILER})
if(IS_ICPC)
	set(CMAKE_AR "xiar" CACHE STRING "Intel archiver" FORCE)
	set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ipo -no-prec-div -xP -DNDEBUG -g" 
	CACHE STRING "Flags used by the C++ compiler during release builds." FORCE)
	set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING
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

function(add_shared_and_static_library name)
	# remove <name> from ARGV
	list(REMOVE_AT ARGV 0)
	add_library(${name} SHARED ${ARGV})
	add_library(${name}_static STATIC ${ARGV})
	set_target_properties(${name}_static PROPERTIES OUTPUT_NAME "${name}")
	# needed for MS Windows, see:
	# http://www.cmake.org/Wiki/CMake_FAQ#How_do_I_make_my_shared_and_static_libraries_have_the_same_root_name.2C_but_different_suffixes.3F
	set_target_properties(${name}_static PROPERTIES PREFIX "lib")
endfunction(add_shared_and_static_library)
