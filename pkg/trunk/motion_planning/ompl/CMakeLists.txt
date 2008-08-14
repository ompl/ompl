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
