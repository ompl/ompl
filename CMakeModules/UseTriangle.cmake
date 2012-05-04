include(FindPackageHandleStandardArgs)
find_library (TRIANGLE_LIBRARY NAMES triangle TRIANGLE DOC "Location of the Triangle package")
find_path (TRIANGLE_INCLUDE_DIR triangle.h PATH_SUFFIXES triangle)

# TODO: Fail if triangle is not found, or perhaps compile without TriangularDecompositions
if (TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
    find_package_handle_standard_args(triangle DEFAULT_MSG TRIANGLE_LIBRARY TRIANGLE_INCLUDE_DIR)
    message (STATUS "Triangle package found.")
else (TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
    message (STATUS "Triangle package not found.")
endif (TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)

find_library(TRIANGLE_LIBRARY triangle DOC "Location of Triangle library")
find_path(TRIANGLE_INCLUDE_DIR triangle.h PATH_SUFFIXES triangle
   DOC "Location of Triangle header file directory")

if(TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
  #if already installed, show that library and headers were found
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(triangle DEFAULT_MSG TRIANGLE_LIBRARY TRIANGLE_INCLUDE_DIR)
else(TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
    message(FATAL_ERROR "Triangle is not found")
endif(TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
