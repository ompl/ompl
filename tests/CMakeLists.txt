# configure location of resources
file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
configure_file("${TEST_RESOURCES_DIR}/config.h.in" "${TEST_RESOURCES_DIR}/config.h")

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
add_ompl_test(test_planner_data base/planner_data.cpp)

# Test kinematic motion planners on a 2D map
add_ompl_test(test_2dmap_geometric geometric/2dmap/2dmap.cpp)
add_ompl_test(test_2dmap_geometric_simple geometric/2dmap/2dmap_simple.cpp)
add_ompl_test(test_2dmap_ik geometric/2dmap/2dmap_ik.cpp)

# Test planning with controls on a 2D map
add_ompl_test(test_2dmap_control control/2dmap/2dmap.cpp)

# Python unit tests
if(OMPL_BUILD_PYTESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../py-bindings/bindings")
    add_ompl_python_test(util/test_util.py)
    add_ompl_python_test(base/test_base.py)
    add_ompl_python_test(geometric/test_geometric.py)
    add_ompl_python_test(geometric/test_geometric_compoundstate.py)
    add_ompl_python_test(control/test_control.py)

    # test the python function to boost::function conversion utility functions
    include_directories(${PYTHON_INCLUDE_DIRS})
    add_library(py_boost_function MODULE util/test_py_boost_function.cpp)
    target_link_libraries(py_boost_function ompl ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARIES})
    get_target_property(PYBOOSTFUNCTION_NAME py_boost_function LOCATION)
    add_custom_command(TARGET py_boost_function POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${PYBOOSTFUNCTION_NAME}"
        "${CMAKE_CURRENT_SOURCE_DIR}/util"
        WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
    add_ompl_python_test(util/test_py_boost_function.py)
endif()
