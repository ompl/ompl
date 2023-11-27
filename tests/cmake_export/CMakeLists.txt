cmake_minimum_required(VERSION 3.12)
project(ompl_cmake_export LANGUAGES CXX)

find_package(ompl REQUIRED)
add_executable(main main.cpp)
target_link_libraries(main PRIVATE ompl::ompl)
