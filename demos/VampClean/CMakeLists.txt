cmake_minimum_required(VERSION 3.5)

# Check if VAMP is available
if(NOT VAMP_FOUND)
    message(STATUS "VAMP not found, skipping VampClean demos")
    return()
endif()

# Include directories
include_directories(${VAMP_INCLUDE_DIRS})

# Demo executable
add_executable(ompl_vamp_clean_demo demos/SimpleVampDemo.cpp)
target_link_libraries(ompl_vamp_clean_demo 
    ${OMPL_LIBRARIES} 
    ${VAMP_LIBRARIES}
)

# Benchmark example
add_executable(ompl_vamp_clean_benchmark examples/BenchmarkExample.cpp)
target_link_libraries(ompl_vamp_clean_benchmark 
    ${OMPL_LIBRARIES} 
    ${VAMP_LIBRARIES}
)

# Set C++ standard
set_target_properties(ompl_vamp_clean_demo PROPERTIES CXX_STANDARD 17)
set_target_properties(ompl_vamp_clean_benchmark PROPERTIES CXX_STANDARD 17)

# Install targets
install(TARGETS ompl_vamp_clean_demo ompl_vamp_clean_benchmark
    DESTINATION bin
    COMPONENT ompl) 