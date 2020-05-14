find_package(Boost 1.58 QUIET COMPONENTS unit_test_framework)
if(Boost_UNIT_TEST_FRAMEWORK_LIBRARY)
    option(OMPL_BUILD_TESTS "Build OMPL tests" ON)
else()
    option(OMPL_BUILD_TESTS "Build OMPL tests" OFF)
endif()
add_feature_info(OMPL_BUILD_TESTS "${OMPL_BUILD_TESTS}" "Whether to build the unit tests.")

if (OMPL_BUILD_TESTS)
    file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
    add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

    # Test datastructures implementation
    add_ompl_test(test_adjacency_list datastructures/adjacency_list.cpp)
    add_ompl_test(test_heap datastructures/heap.cpp)
    add_ompl_test(test_grid datastructures/grid.cpp)
    add_ompl_test(test_gridb datastructures/gridb.cpp)
    add_ompl_test(test_nearestneighbors datastructures/nearestneighbors.cpp)
    if(flann_FOUND)
        target_link_libraries(test_nearestneighbors ${FLANN_LIBRARIES})
    endif()
    add_ompl_test(test_pdf datastructures/pdf.cpp)

    # Test utilities
    add_ompl_test(test_random util/random/random.cpp)
    # optimization flags make this test fail
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_ompl_test(test_machine_specs benchmark/machine_specs.cpp)
    endif()

    # Test base code
    add_ompl_test(test_halton_sampling base/halton_deterministic_sampling.cpp)
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
    add_ompl_test(test_2dpath_simplifying geometric/2d/2dpath_simplifying.cpp)

    # Test constrained planning
    add_ompl_test(test_constraint_sphere geometric/constraint/test_sphere.cpp)

    # Test planning with controls on a 2D map
    add_ompl_test(test_2dmap_control control/2dmap/2dmap.cpp)
    add_ompl_test(test_planner_data_control control/planner_data.cpp)

    # Test planning via MORSE extension
    if(OMPL_EXTENSION_MORSE)
        add_ompl_test(test_morse_extension extensions/morse/morse_plan.cpp)
    endif(OMPL_EXTENSION_MORSE)

    # Python unit tests
    if(PYTHON_FOUND AND OMPL_BUILD_PYTESTS)
        if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/../py-bindings/bindings")
            add_ompl_python_test(util/test_util.py)
            add_ompl_python_test(base/test_base.py)
            add_ompl_python_test(geometric/test_geometric.py)
            add_ompl_python_test(geometric/test_geometric_compoundstate.py)
            add_ompl_python_test(control/test_control.py)
        endif()

        # test the python function to std::function conversion utility functions
        include_directories(${PYTHON_INCLUDE_DIRS})
        add_library(py_std_function MODULE util/test_py_std_function.cpp)
        target_link_libraries(py_std_function ompl ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARIES})
        if(WIN32)
            set_target_properties(py_std_function PROPERTIES OUTPUT_NAME py_std_function SUFFIX .pyd)
        endif(WIN32)
        add_custom_command(TARGET py_std_function POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:py_std_function>"
            "${CMAKE_CURRENT_SOURCE_DIR}/util"
            WORKING_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
        add_ompl_python_test(util/test_py_std_function.py)
    endif()

    add_subdirectory(regression_tests)
endif()
