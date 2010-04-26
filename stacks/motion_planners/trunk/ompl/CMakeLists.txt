cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_add_boost_directories()
include_directories(${PROJECT_SOURCE_DIR}/code)
set(CMAKE_BUILD_TYPE Release)
rosbuild_add_library(ompl code/ompl/datastructures/src/RandomNumbers.cpp 
                          code/ompl/datastructures/src/JLMatrix.cpp 
                          code/ompl/datastructures/src/SearchGrid.cpp 
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
			  code/ompl/extension/kinematic/extension/rrt/src/RRTConnect.cpp
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

rosbuild_add_gtest(test_searchgrid code/tests/datastructures/searchgrid/searchgrid.cpp)
target_link_libraries(test_searchgrid ompl)
rosbuild_link_boost(test_searchgrid thread)

rosbuild_add_gtest(test_jlmatrix code/tests/datastructures/jlmatrix/jlmatrix.cpp)
target_link_libraries(test_jlmatrix ompl)
rosbuild_link_boost(test_jlmatrix thread)

rosbuild_add_gtest(test_random code/tests/datastructures/random/random.cpp)
target_link_libraries(test_random ompl)
rosbuild_link_boost(test_random thread)

# Test compilation
rosbuild_add_gtest(test_compile code/tests/compile/compile.cpp)
target_link_libraries(test_compile ompl)
rosbuild_link_boost(test_compile thread)
