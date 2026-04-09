find_package(Boost 1.68 QUIET COMPONENTS unit_test_framework)
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

    if(NOT MSVC)
      add_ompl_test(test_grid datastructures/grid.cpp)
      add_ompl_test(test_gridb datastructures/gridb.cpp)
      add_ompl_test(test_nearestneighbors datastructures/nearestneighbors.cpp)
    endif()

    if(OMPL_HAVE_FLANN)
        # FLANN uses std::random_shuffle in a header file which is removed in C++17
        # https://github.com/flann-lib/flann/issues/387
        set_target_properties(test_nearestneighbors PROPERTIES CXX_STANDARD 14)
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
    add_ompl_test(test_planner_data base/planner_data.cpp)

    if(NOT MSVC)
      add_ompl_test(test_ptc base/ptc.cpp)
    endif()

    # Test kinematic motion planners in 2D environments
    # add_ompl_test(test_2denvs_geometric geometric/2d/2denvs.cpp) # Test is brittle for github runners
    add_ompl_test(test_2dmap_geometric_simple geometric/2d/2dmap_simple.cpp)
    add_ompl_test(test_2dmap_ik geometric/2d/2dmap_ik.cpp)
    add_ompl_test(test_2dcircles_opt_geometric geometric/2d/2dcircles_optimize.cpp)
    add_ompl_test(test_2dpath_simplifying geometric/2d/2dpath_simplifying.cpp)

    # Test DIRT planner
    add_ompl_test(test_dirt geometric/dirt/test_dirt.cpp)

    # Test constrained planning
    add_ompl_test(test_constraint_sphere geometric/constraint/test_sphere.cpp)

    # Test space-time planning
    add_ompl_test(test_spacetime_STRRTstar geometric/spacetime/STRRTstar_test.cpp)

    # Test planning with controls on a 2D map
    add_ompl_test(test_2dmap_control control/2dmap/2dmap.cpp)
    add_ompl_test(test_planner_data_control control/planner_data.cpp)

    # Test experience based planning
    add_ompl_test(test_experience_planning tools/test_experience_planning.cpp)

    # Test VAMP integration
    if(OMPL_HAVE_VAMP)
        add_ompl_test(test_vamp_integration vamp/test_vamp_integration.cpp)
        add_ompl_test(test_vamp_planning vamp/vamp_test.cpp)
        if(TARGET vamp_cpp)
            target_link_libraries(test_vamp_integration vamp_cpp)
            target_link_libraries(test_vamp_planning vamp_cpp)

            # Apply VAMP SIMD flags to the test to ensure SIMD functionality works
            if(DEFINED VAMP_SIMD_FLAGS)
                target_compile_options(test_vamp_integration PRIVATE ${VAMP_SIMD_FLAGS})
                message(STATUS "Applied VAMP SIMD flags to test_vamp_integration: ${VAMP_SIMD_FLAGS}")
            endif()

            message(STATUS "VAMP integration test configured with vamp_cpp target and SIMD flags")
        else()
            message(WARNING "OMPL_HAVE_VAMP is TRUE but vamp_cpp target not found")
        endif()
    endif()
    add_subdirectory(regression_tests)
endif()
