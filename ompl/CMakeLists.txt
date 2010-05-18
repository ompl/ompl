option(ROS "Build against ROS" FALSE)

if(NOT ROS)

  cmake_minimum_required(VERSION 2.6)
  project(ompl CXX C)
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

endif(NOT ROS)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

set(OMPL_CODE ${PROJECT_SOURCE_DIR}/code)
include_directories(${OMPL_CODE})

set(OMPL_SOURCE_CODE ${OMPL_CODE}/ompl/util/src/RandomNumbers.cpp 
                     ${OMPL_CODE}/ompl/util/src/Console.cpp
                     ${OMPL_CODE}/ompl/datastructures/src/SearchGrid.cpp
		     ${OMPL_CODE}/ompl/base/src/SpaceInformation.cpp
		     ${OMPL_CODE}/ompl/base/src/ProblemDefinition.cpp
		     ${OMPL_CODE}/ompl/base/src/L2SquareStateDistanceEvaluator.cpp
		     ${OMPL_CODE}/ompl/base/src/LinearProjectionEvaluator.cpp
		     ${OMPL_CODE}/ompl/base/src/OrthogonalProjectionEvaluator.cpp
		     ${OMPL_CODE}/ompl/base/src/StateSampler.cpp
		     ${OMPL_CODE}/ompl/base/src/StateSamplerArray.cpp
		     ${OMPL_CODE}/ompl/base/src/UniformStateSampler.cpp
		     ${OMPL_CODE}/ompl/base/src/GoalRegion.cpp
		     ${OMPL_CODE}/ompl/base/src/GoalState.cpp
		     ${OMPL_CODE}/ompl/base/src/Planner.cpp
		     ${OMPL_CODE}/ompl/kinematic/src/PathKinematic.cpp
		     ${OMPL_CODE}/ompl/kinematic/src/SpaceInformationKinematic.cpp
		     ${OMPL_CODE}/ompl/kinematic/src/PathSmootherKinematic.cpp
		     ${OMPL_CODE}/ompl/kinematic/src/LinearStateInterpolatorKinematic.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/ik/src/HCIK.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/ik/src/GAIK.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/rrt/src/RRT.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/rrt/src/pRRT.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/rrt/src/LazyRRT.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/rrt/src/RRTConnect.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/est/src/EST.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/sbl/src/SBL.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/sbl/src/pSBL.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/kpiece/src/KPIECE1.cpp
		     ${OMPL_CODE}/ompl/kinematic/planners/kpiece/src/LBKPIECE1.cpp
		     ${OMPL_CODE}/ompl/dynamic/src/ControlSampler.cpp
		     ${OMPL_CODE}/ompl/dynamic/src/PathDynamic.cpp
		     ${OMPL_CODE}/ompl/dynamic/src/SpaceInformationControls.cpp
		     ${OMPL_CODE}/ompl/dynamic/src/UniformControlSampler.cpp
		     ${OMPL_CODE}/ompl/dynamic/src/SpaceInformationControlsIntegrator.cpp
		     ${OMPL_CODE}/ompl/dynamic/src/SpaceInformationControlsPhysics.cpp
		     ${OMPL_CODE}/ompl/dynamic/planners/rrt/src/RRT.cpp
		     ${OMPL_CODE}/ompl/dynamic/planners/kpiece/src/KPIECE1.cpp)

if(ROS)  

  rosbuild_add_boost_directories()
  rosbuild_add_library(ompl ${OMPL_SOURCE_CODE})
  rosbuild_add_compile_flags(ompl -Wextra)
  rosbuild_link_boost(ompl thread date_time)
  option(BUILD_TESTS "Build ompl tests" TRUE)
  
else(ROS)
  find_package(Boost COMPONENTS date_time thread REQUIRED)
  add_library(ompl SHARED ${OMPL_SOURCE_CODE})
  add_definitions(-W -Wall -Wextra)
  target_link_libraries(ompl
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    )
  
  find_package(GTest)
  option(BUILD_TESTS "Build ompl tests" ${GTEST_FOUND})

endif(ROS)

### Tests


if(BUILD_TESTS)

  if(ROS)
    
    macro(add_ompl_test test_name)
      rosbuild_add_gtest(${ARGV})
      target_link_libraries(${test_name} ompl)
      rosbuild_link_boost(test_2dmap_kinematic thread date_time)			
    endmacro(add_ompl_test)

  else(ROS)
    
    enable_testing()
    
    include_directories(${GTEST_INCLUDE_DIRS})
    
    macro(add_ompl_test test_name)
      add_executable(${ARGV})
      target_link_libraries(${test_name} 
        ompl
	${Boost_THREAD_LIBRARY}
	${Boost_DATE_TIME_LIBRARY} ${GTEST_LIBRARIES})
      add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
    endmacro(add_ompl_test)
    
  endif(ROS)
  
  # Test kinematic motion planners on a 2D map
  add_ompl_test(test_2dmap_kinematic ${OMPL_CODE}/tests/kinematic/2dmap/2dmap.cpp)

  # Test dynamic motion planners on a 2D map
  add_ompl_test(test_2dmap_dynamic ${OMPL_CODE}/tests/dynamic/2dmap/2dmap.cpp)

  # Test datastructures implementation
  add_ompl_test(test_heap ${OMPL_CODE}/tests/datastructures/heap/heap.cpp)
  add_ompl_test(test_grid ${OMPL_CODE}/tests/datastructures/grid/grid.cpp)
  add_ompl_test(test_gridb ${OMPL_CODE}/tests/datastructures/grid/gridb.cpp)
  add_ompl_test(test_searchgrid ${OMPL_CODE}/tests/datastructures/searchgrid/searchgrid.cpp)

  # Test utilities
  add_ompl_test(test_random ${OMPL_CODE}/tests/util/random/random.cpp)

  # Test compilation
  add_ompl_test(test_compile ${OMPL_CODE}/tests/compile/compile.cpp)

endif(BUILD_TESTS)
