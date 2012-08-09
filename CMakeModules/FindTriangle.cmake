# Finds if Triangle is installed and determines the locations of the
# include headers and library files.
#
# Written by Matt Maly.

include(FindPackageHandleStandardArgs)

find_library(TRIANGLE_LIBRARY triangle DOC "Location of Triangle library")
find_path(TRIANGLE_INCLUDE_DIR triangle.h PATH_SUFFIXES triangle
   DOC "Location of Triangle header file directory")
find_package_handle_standard_args(triangle DEFAULT_MSG TRIANGLE_LIBRARY TRIANGLE_INCLUDE_DIR)
mark_as_advanced(TRIANGLE_LIBRARY TRIANGLE_INCLUDE_DIR)

#if(TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
  #if already installed, show that library and headers were found
#    include(FindPackageHandleStandardArgs)
#    find_package_handle_standard_args(triangle DEFAULT_MSG TRIANGLE_LIBRARY TRIANGLE_INCLUDE_DIR)
#else(TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
  #otherwise, print an error and exit
  #TODO: if triangle not found, compile ompl without triangular decompositions
#    message(FATAL_ERROR "Triangle is not found")
#endif(TRIANGLE_LIBRARY AND TRIANGLE_INCLUDE_DIR)
