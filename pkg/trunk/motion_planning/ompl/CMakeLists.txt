cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ompl)
rospack_add_boost_directories()
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(ompl code/ompl/base/util/src/time.cpp
			 code/ompl/base/util/src/output.cpp
			 code/ompl/base/util/src/random_utils.cpp
			 code/ompl/base/src/Planner.cpp
			 code/ompl/extension/samplingbased/kinematic/src/SpaceInformationKinematic.cpp
			 code/ompl/extension/samplingbased/kinematic/src/PathSmootherKinematic.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/ik/src/GAIK.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/rrt/src/RRT.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/rrt/src/LazyRRT.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/est/src/EST.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/sbl/src/SBL.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/sbl/src/IKSBL.cpp
			 )
rospack_add_compile_flags(ompl -Wextra)

### Tests

# Test datastructures implementation
rospack_add_gtest(test_grid code/tests/datastructures/grid/grid.cpp)
target_link_libraries(test_grid ompl)
rospack_link_boost(test_grid thread)

# Thest the motion planners on a 2D map
rospack_add_gtest(test_2dmap code/tests/samplingbased/kinematic/2dmap/2dmap.cpp)
target_link_libraries(test_2dmap ompl)
rospack_link_boost(test_2dmap thread)

