# configure location of resources
file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
if(WIN32)
    # Correct directory separator for Windows
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
endif(WIN32)
configure_file("${TEST_RESOURCES_DIR}/config.h.in" "${TEST_RESOURCES_DIR}/config.h")

# Disable teamcity reporting for tests by default
option(OMPL_TESTS_TEAMCITY "Enable unit test reporting to TeamCity" OFF)
file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CURRENT_DIR)
configure_file("${CURRENT_DIR}/BoostTestTeamCityReporter.h.in" "${CURRENT_DIR}/BoostTestTeamCityReporter.h")

# Test datastructures implementation
add_ompl_test(test_heap datastructures/heap.cpp)
add_ompl_test(test_grid datastructures/grid.cpp)
add_ompl_test(test_gridb datastructures/gridb.cpp)
add_ompl_test(test_nearestneighbors datastructures/nearestneighbors.cpp)
add_ompl_test(test_pdf datastructures/pdf.cpp)

# Test utilities
add_ompl_test(test_random util/random/random.cpp)
add_ompl_test(test_machine_specs benchmark/machine_specs.cpp)

# Test base code
add_ompl_test(test_state_operations base/state_operations.cpp)
add_ompl_test(test_state_spaces base/state_spaces.cpp)
add_ompl_test(test_state_storage base/state_storage.cpp)
add_ompl_test(test_ptc base/ptc.cpp)
add_ompl_test(test_planner_data base/planner_data.cpp)

# Test kinematic motion planners in 2D environments
add_ompl_test(test_2denvs_geometric geometric/2d/2denvs.cpp)
add_ompl_test(test_2dmap_geometric_simple geometric/2d/2dmap_simple.cpp)
add_ompl_test(test_2dmap_ik geometric/2d/2dmap_ik.cpp)
add_ompl_test(test_2dcircles_opt_geometric geometric/2d/2dcircles_optimize.cpp)

# Test planning with controls on a 2D map
add_ompl_test(test_2dmap_control control/2dmap/2dmap.cpp)
add_ompl_test(test_planner_data_control control/planner_data.cpp)

# Test planning via MORSE extension
if(OMPL_EXTENSION_MORSE)
    add_ompl_test(test_morse_extension extensions/morse/morse_plan.cpp)
endif(OMPL_EXTENSION_MORSE)

# Python unit tests
if(PYTHON_FOUND AND OMPL_BUILD_PYTESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../py-bindings/bindings")
    add_ompl_python_test(util/test_util.py)
    add_ompl_python_test(base/test_base.py)
    add_ompl_python_test(geometric/test_geometric.py)
    add_ompl_python_test(geometric/test_geometric_compoundstate.py)
    add_ompl_python_test(control/test_control.py)

    # test the python function to boost::function conversion utility functions
    include_directories(${PYTHON_INCLUDE_DIRS})
    add_library(py_boost_function MODULE util/test_py_boost_function.cpp)
    target_link_libraries(py_boost_function ompl ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARIES})
    if(WIN32)
        set_target_properties(py_boost_function PROPERTIES OUTPUT_NAME py_boost_function SUFFIX .pyd)
    endif(WIN32)
    add_custom_command(TARGET py_boost_function POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:py_boost_function>"
        "${CMAKE_CURRENT_SOURCE_DIR}/util"
        WORKING_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
    add_ompl_python_test(util/test_py_boost_function.py)
endif()

add_subdirectory(regression_tests)
