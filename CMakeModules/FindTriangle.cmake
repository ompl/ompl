# Finds if Triangle is installed and determines the locations of the
# include headers and library files.
#
# Written by Matt Maly.

include(FindPackageHandleStandardArgs)

find_library(TRIANGLE_LIBRARY triangle DOC "Location of Triangle library")
find_path(TRIANGLE_INCLUDE_DIR triangle.h PATH_SUFFIXES triangle
    DOC "Location of Triangle header file directory")
find_package_handle_standard_args(Triangle DEFAULT_MSG TRIANGLE_LIBRARY TRIANGLE_INCLUDE_DIR)
mark_as_advanced(TRIANGLE_LIBRARY TRIANGLE_INCLUDE_DIR)
