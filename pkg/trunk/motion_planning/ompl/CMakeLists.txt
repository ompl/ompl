cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(ompl)
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(ompl code/ompl/base/util/src/time.cpp
			 code/ompl/base/util/src/random.cpp
			 code/ompl/extension/samplingbased/kinematic/src/SpaceInformationKinematic.cpp
			 code/ompl/extension/samplingbased/kinematic/src/KinematicPathSmoother.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/rrt/src/RRT.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/rrt/src/LazyRRT.cpp
			 code/ompl/extension/samplingbased/kinematic/extension/sbl/src/SBL.cpp
			 )

rospack_add_executable(test_grid code/examples/samplingbased/kinematic/grid/grid.cpp)
target_link_libraries(test_grid ompl)
rospack_add_link_flags(test_grid -lpthread -lrt)
