cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_add_boost_directories()
include_directories(${PROJECT_SOURCE_DIR}/code)
set(CMAKE_BUILD_TYPE Release)
rosbuild_add_library(ompl code/ompl/datastructures/src/RandomNumbers.cpp 
                          code/ompl/datastructures/src/SearchGrid.cpp
			  code/ompl/base/src/SpaceInformation.cpp
			  code/ompl/base/src/L2SquareStateDistanceEvaluator.cpp
			  code/ompl/base/src/LinearProjectionEvaluator.cpp
			  code/ompl/base/src/OrthogonalProjectionEvaluator.cpp
			  code/ompl/base/src/StateSampler.cpp
			  code/ompl/base/src/StateSamplerArray.cpp
			  code/ompl/base/src/UniformStateSampler.cpp
			  code/ompl/base/src/GoalRegion.cpp
			  code/ompl/base/src/GoalState.cpp
			  code/ompl/base/src/Planner.cpp
			  code/ompl/kinematic/src/PathKinematic.cpp
			  code/ompl/kinematic/src/SpaceInformationKinematic.cpp
			  code/ompl/kinematic/src/PathSmootherKinematic.cpp
			  code/ompl/kinematic/src/LinearStateInterpolatorKinematic.cpp
			  code/ompl/kinematic/planners/ik/src/HCIK.cpp
			  code/ompl/kinematic/planners/ik/src/GAIK.cpp
			  code/ompl/kinematic/planners/rrt/src/RRT.cpp
			  code/ompl/kinematic/planners/rrt/src/pRRT.cpp
			  code/ompl/kinematic/planners/rrt/src/LazyRRT.cpp
			  code/ompl/kinematic/planners/rrt/src/RRTConnect.cpp
			  code/ompl/kinematic/planners/est/src/EST.cpp
			  code/ompl/kinematic/planners/sbl/src/SBL.cpp
			  code/ompl/kinematic/planners/sbl/src/pSBL.cpp
			  code/ompl/kinematic/planners/kpiece/src/KPIECE1.cpp
			  code/ompl/kinematic/planners/kpiece/src/LBKPIECE1.cpp
			  code/ompl/dynamic/src/ControlSampler.cpp
			  code/ompl/dynamic/src/PathDynamic.cpp
			  code/ompl/dynamic/src/SpaceInformationControls.cpp
			  code/ompl/dynamic/src/UniformControlSampler.cpp
			  code/ompl/dynamic/src/SpaceInformationControlsIntegrator.cpp
			  code/ompl/dynamic/src/SpaceInformationControlsPhysics.cpp
			  code/ompl/dynamic/planners/rrt/src/RRT.cpp
			  code/ompl/dynamic/planners/kpiece/src/KPIECE1.cpp
			  )
rosbuild_add_compile_flags(ompl -Wextra)
rosbuild_link_boost(ompl thread)

### Tests


# Test kinematic motion planners on a 2D map
rosbuild_add_gtest(test_2dmap_kinematic code/tests/kinematic/2dmap/2dmap.cpp)
target_link_libraries(test_2dmap_kinematic ompl)
rosbuild_link_boost(test_2dmap_kinematic thread)

# Test dynamic motion planners on a 2D map
rosbuild_add_gtest(test_2dmap_dynamic code/tests/dynamic/2dmap/2dmap.cpp)
target_link_libraries(test_2dmap_dynamic ompl)
rosbuild_link_boost(test_2dmap_dynamic thread)

# Test datastructures implementation

rosbuild_add_gtest(test_heap code/tests/datastructures/heap/heap.cpp)
target_link_libraries(test_heap ompl)
rosbuild_link_boost(test_heap thread)

rosbuild_add_gtest(test_grid code/tests/datastructures/grid/grid.cpp)
target_link_libraries(test_grid ompl)
rosbuild_link_boost(test_grid thread)

rosbuild_add_gtest(test_gridb code/tests/datastructures/grid/gridb.cpp)
target_link_libraries(test_gridb ompl)
rosbuild_link_boost(test_gridb thread)

rosbuild_add_gtest(test_random code/tests/datastructures/random/random.cpp)
target_link_libraries(test_random ompl)
rosbuild_link_boost(test_random thread)

rosbuild_add_gtest(test_searchgrid code/tests/datastructures/searchgrid/searchgrid.cpp)
target_link_libraries(test_searchgrid ompl)
rosbuild_link_boost(test_searchgrid thread)

# Test compilation
rosbuild_add_gtest(test_compile code/tests/compile/compile.cpp)
target_link_libraries(test_compile ompl)
rosbuild_link_boost(test_compile thread)
