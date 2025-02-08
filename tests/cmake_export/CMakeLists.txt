cmake_minimum_required(VERSION 3.12)
project(ompl_cmake_export LANGUAGES CXX)

include("${CMAKE_CURRENT_LIST_DIR}/../../CMakeModules/CompilerSettings.cmake")

find_package(ompl REQUIRED)
add_executable(main main.cpp)
target_link_libraries(main PRIVATE ompl::ompl)
