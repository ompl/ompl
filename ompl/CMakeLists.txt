cmake_minimum_required(VERSION 2.6)
project(ompl CXX C)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(Boost COMPONENTS date_time thread REQUIRED)

set(OMPL_CODE ${PROJECT_SOURCE_DIR}/code)
include_directories(${OMPL_CODE})
add_definitions(-W -Wall -Wextra)

add_library(ompl SHARED ${OMPL_CODE}/ompl/util/src/RandomNumbers.cpp 
			${OMPL_CODE}/ompl/util/src/Console.cpp
                        ${OMPL_CODE}/ompl/datastructures/src/SearchGrid.cpp
			${OMPL_CODE}/ompl/base/src/SpaceInformation.cpp
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
			${OMPL_CODE}/ompl/dynamic/planners/kpiece/src/KPIECE1.cpp
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
  ADD_OMPL_TEST(test_2dmap_kinematic ${OMPL_CODE}/tests/kinematic/2dmap/2dmap.cpp)

  # Test dynamic motion planners on a 2D map
  ADD_OMPL_TEST(test_2dmap_dynamic ${OMPL_CODE}/tests/dynamic/2dmap/2dmap.cpp)

  # Test datastructures implementation
  ADD_OMPL_TEST(test_heap ${OMPL_CODE}/tests/datastructures/heap/heap.cpp)
  ADD_OMPL_TEST(test_grid ${OMPL_CODE}/tests/datastructures/grid/grid.cpp)
  ADD_OMPL_TEST(test_gridb ${OMPL_CODE}/tests/datastructures/grid/gridb.cpp)
  ADD_OMPL_TEST(test_searchgrid ${OMPL_CODE}/tests/datastructures/searchgrid/searchgrid.cpp)

  # Test utilities
  ADD_OMPL_TEST(test_random ${OMPL_CODE}/tests/util/random/random.cpp)

  # Test compilation
  ADD_OMPL_TEST(test_compile ${OMPL_CODE}/tests/compile/compile.cpp)

endif(BUILD_TESTS)
