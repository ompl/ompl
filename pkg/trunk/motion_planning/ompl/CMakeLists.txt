cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(ompl)
rospack_add_library(ompl code/ompl/extension/samplingbased/kinematic/src/SpaceInformationKinematic.cpp)
