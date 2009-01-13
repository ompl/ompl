cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ompl)
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(ompl code/ompl/base/util/src/time.cpp
			 code/ompl/base/util/src/output.cpp
			 code/ompl/extension/samplingbased/kinematic/src/SpaceInformationKinematic.cpp
			 code/ompl/extension/samplingbased/kinematic/src/PathSmootherKinematic.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/rrt/src/RRT.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/rrt/src/LazyRRT.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/est/src/EST.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/sbl/src/SBL.cpp
			 )
rospack_add_compile_flags(ompl -Wextra)

# Tests
rospack_add_gtest(test_grid code/examples/samplingbased/kinematic/grid/grid.cpp)
target_link_libraries(test_grid ompl)
