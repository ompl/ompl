cmake_minimum_required(VERSION 2.6)
project(ompl CXX C)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(Boost COMPONENTS date_time thread REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/code)
add_definitions(-W -Wall -Wextra)

add_library(ompl SHARED code/ompl/util/src/RandomNumbers.cpp 
			code/ompl/util/src/Console.cpp
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

target_link_libraries(ompl
  ${Boost_THREAD_LIBRARY}
  ${Boost_DATE_TIME_LIBRARY}
)

### Tests

find_package(GTest)
option(BUILD_TESTS "Build ompl tests" ${GTEST_FOUND})

if(BUILD_TESTS)
  enable_testing()
	
  include_directories(${GTEST_INCLUDE_DIRS})
  
  macro(ADD_OMPL_TEST test_name)
    add_executable(${ARGV})
    target_link_libraries(${test_name} 
                          ompl
			  ${Boost_THREAD_LIBRARY}
			  ${Boost_DATE_TIME_LIBRARY} ${GTEST_LIBRARIES})
    add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
  endmacro(ADD_OMPL_TEST)


  # Test kinematic motion planners on a 2D map
  ADD_OMPL_TEST(test_2dmap_kinematic code/tests/kinematic/2dmap/2dmap.cpp)

  # Test dynamic motion planners on a 2D map
  ADD_OMPL_TEST(test_2dmap_dynamic code/tests/dynamic/2dmap/2dmap.cpp)

  # Test datastructures implementation
  ADD_OMPL_TEST(test_heap code/tests/datastructures/heap/heap.cpp)
  ADD_OMPL_TEST(test_grid code/tests/datastructures/grid/grid.cpp)
  ADD_OMPL_TEST(test_gridb code/tests/datastructures/grid/gridb.cpp)
  ADD_OMPL_TEST(test_searchgrid code/tests/datastructures/searchgrid/searchgrid.cpp)

  # Test utilities
  ADD_OMPL_TEST(test_random code/tests/util/random/random.cpp)

  # Test compilation
  ADD_OMPL_TEST(test_compile code/tests/compile/compile.cpp)

endif(BUILD_TESTS)
