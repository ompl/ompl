if(OMPL_BUILD_TESTS)

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

    # Test kinematic motion planners on a 2D map
    add_ompl_test(test_2dmap_geometric geometric/2dmap/2dmap.cpp)
    add_ompl_test(test_2dmap_geometric_simple geometric/2dmap/2dmap_simple.cpp)
    add_ompl_test(test_2dmap_ik geometric/2dmap/2dmap_ik.cpp)

    # Test planning with controls on a 2D map
    add_ompl_test(test_2dmap_control control/2dmap/2dmap.cpp)

    # Python unit tests
    if(OMPL_BUILD_PYTESTS)
      add_ompl_python_test(util/test_util.py)
      add_ompl_python_test(base/test_base.py)
      add_ompl_python_test(geometric/test_geometric.py)
      add_ompl_python_test(geometric/test_geometric_compoundstate.py)
      add_ompl_python_test(control/test_control.py)
    endif()

endif(OMPL_BUILD_TESTS)
