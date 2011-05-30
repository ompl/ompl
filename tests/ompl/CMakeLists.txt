
# Test datastructures implementation
add_ompl_test(test_heap datastructures/heap.cpp)
add_ompl_test(test_grid datastructures/grid.cpp)
add_ompl_test(test_gridb datastructures/gridb.cpp)
add_ompl_test(test_nearestneighbors datastructures/nearestneighbors.cpp)

# Test utilities
add_ompl_test(test_random util/random/random.cpp)
add_ompl_test(test_memory util/memory/memory.cpp)

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
