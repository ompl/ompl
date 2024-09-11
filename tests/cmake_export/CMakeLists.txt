cmake_minimum_required(VERSION 3.12)
project(ompl_cmake_export LANGUAGES CXX)

find_package(ompl REQUIRED)

message(STATUS "Legacy Variables:")
message("* OMPL_FOUND: ${OMPL_FOUND}")
message("* ompl_FOUND: ${ompl_FOUND}")
message("* OMPL_INCLUDE_DIRS: ${OMPL_INCLUDE_DIRS}")
message("* OMPL_LIBRARIES: ${OMPL_LIBRARIES}")
message("* OMPLAPP_LIBRARIES: ${OMPLAPP_LIBRARIES}")
message("* OMPL_VERSION: ${OMPL_VERSION}")
message("* OMPL_MAJOR_VERSION: ${OMPL_MAJOR_VERSION}")
message("* OMPL_MINOR_VERSION: ${OMPL_MINOR_VERSION}")
message("* OMPL_PATCH_VERSION: ${OMPL_PATCH_VERSION}")

include(CMakePrintHelpers)
cmake_print_properties(
    TARGETS ompl::ompl PROPERTIES
    LOCATION INTERFACE_INCLUDE_DIRECTORIES INCLUDE_DIRECTORIES)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE ompl::ompl)
