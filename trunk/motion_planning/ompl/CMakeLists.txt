cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ompl)
rospack_add_boost_directories()
include_directories(${PROJECT_SOURCE_DIR}/code)
set(CMAKE_BUILD_TYPE Release)
rospack_add_library(ompl code/ompl/base/src/RandomNumbers.cpp
			 code/ompl/base/src/SpaceInformation.cpp
			 code/ompl/base/src/StateDistanceEvaluator.cpp
			 code/ompl/base/src/ProjectionEvaluator.cpp
			 code/ompl/base/src/Goal.cpp
			 code/ompl/base/src/Planner.cpp
			 code/ompl/extension/kinematic/src/PathKinematic.cpp
			 code/ompl/extension/kinematic/src/SpaceInformationKinematic.cpp
			 code/ompl/extension/kinematic/src/PathSmootherKinematic.cpp
			 code/ompl/extension/kinematic/extension/ik/src/HCIK.cpp
			 code/ompl/extension/kinematic/extension/ik/src/GAIK.cpp
			 code/ompl/extension/kinematic/extension/rrt/src/RRT.cpp
			 code/ompl/extension/kinematic/extension/rrt/src/pRRT.cpp
			 code/ompl/extension/kinematic/extension/rrt/src/LazyRRT.cpp
			 code/ompl/extension/kinematic/extension/est/src/EST.cpp
			 code/ompl/extension/kinematic/extension/sbl/src/SBL.cpp
			 code/ompl/extension/kinematic/extension/sbl/src/pSBL.cpp
			 code/ompl/extension/kinematic/extension/kpiece/src/KPIECE1.cpp
			 code/ompl/extension/kinematic/extension/kpiece/src/LBKPIECE1.cpp
			 code/ompl/extension/dynamic/src/SpaceInformationControls.cpp
			 code/ompl/extension/dynamic/src/SpaceInformationControlsIntegrator.cpp
			 code/ompl/extension/dynamic/src/SpaceInformationControlsPhysics.cpp
			 code/ompl/extension/dynamic/src/PathDynamic.cpp
			 code/ompl/extension/dynamic/extension/rrt/src/RRT.cpp
			 code/ompl/extension/dynamic/extension/kpiece/src/KPIECE1.cpp
			 )
rospack_add_compile_flags(ompl -Wextra)
rospack_link_boost(ompl thread)

### Tests


# Test kinematic motion planners on a 2D map
rospack_add_gtest(test_2dmap_kinematic code/tests/kinematic/2dmap/2dmap.cpp)
target_link_libraries(test_2dmap_kinematic ompl)
rospack_link_boost(test_2dmap_kinematic thread)

# Test dynamic motion planners on a 2D map
rospack_add_gtest(test_2dmap_dynamic code/tests/dynamic/2dmap/2dmap.cpp)
target_link_libraries(test_2dmap_dynamic ompl)
rospack_link_boost(test_2dmap_dynamic thread)

# Test datastructures implementation
rospack_add_gtest(test_grid code/tests/datastructures/grid/grid.cpp)
target_link_libraries(test_grid ompl)
rospack_link_boost(test_grid thread)

rospack_add_gtest(test_heap code/tests/datastructures/heap/heap.cpp)
target_link_libraries(test_heap ompl)
rospack_link_boost(test_heap thread)

rospack_add_gtest(test_gridb code/tests/datastructures/grid/gridb.cpp)
target_link_libraries(test_gridb ompl)
rospack_link_boost(test_gridb thread)

# Test random numbers
rospack_add_gtest(test_random code/tests/random/random.cpp)
target_link_libraries(test_random ompl)
rospack_link_boost(test_random thread)

# Test compilation
rospack_add_gtest(test_compile code/tests/compile/compile.cpp)
target_link_libraries(test_compile ompl)
rospack_link_boost(test_compile thread)
