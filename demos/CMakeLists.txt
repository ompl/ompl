option(OMPL_BUILD_DEMOS "Build OMPL demos" ON)

if (OMPL_BUILD_DEMOS)

  macro(add_ompl_demo demo_name)
    add_executable(${ARGV})
    target_link_libraries(${demo_name}
      ompl
      ${Boost_FILESYSTEM_LIBRARY}
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_THREAD_LIBRARY}
      ${Boost_DATE_TIME_LIBRARY}
      ${Boost_PROGRAM_OPTIONS_LIBRARY})
  endmacro(add_ompl_demo)

  add_ompl_demo(demo_RigidBodyPlanning RigidBodyPlanning.cpp)
  add_ompl_demo(demo_RigidBodyPlanningWithIK RigidBodyPlanningWithIK.cpp)
  add_ompl_demo(demo_RigidBodyPlanningWithControls RigidBodyPlanningWithControls.cpp)
  add_ompl_demo(demo_RigidBodyPlanningWithIntegrationAndControls RigidBodyPlanningWithIntegrationAndControls.cpp)
  add_ompl_demo(demo_RigidBodyPlanningWithODESolverAndControls RigidBodyPlanningWithODESolverAndControls.cpp)
  add_ompl_demo(demo_StateSampling StateSampling.cpp)
  add_ompl_demo(demo_GeometricCarPlanning GeometricCarPlanning.cpp)
  add_ompl_demo(demo_Point2DPlanning Point2DPlanning.cpp)
  add_ompl_demo(demo_HybridSystemPlanning HybridSystemPlanning.cpp)
  add_ompl_demo(demo_KinematicChainBenchmark KinematicChainBenchmark.cpp)
  add_ompl_demo(demo_HypercubeBenchmark HypercubeBenchmark.cpp)
  aux_source_directory(Koules Koules_SRC)
  add_ompl_demo(demo_Koules ${Koules_SRC})
  add_ompl_demo(demo_PlannerData PlannerData.cpp)

  add_ompl_demo(demo_OptimalPlanning OptimalPlanning.cpp)
  add_ompl_demo(demo_PlannerProgressProperties PlannerProgressProperties.cpp)
  add_ompl_demo(demo_CForestCircleGridBenchmark CForestCircleGridBenchmark.cpp)

  if (OMPL_EXTENSION_OPENDE)
    add_ompl_demo(demo_OpenDERigidBodyPlanning OpenDERigidBodyPlanning.cpp)
  endif()

  if (OMPL_EXTENSION_TRIANGLE)
    add_ompl_demo(demo_TriangulationDemo TriangulationDemo.cpp)
    add_ompl_demo(demo_LTLWithTriangulation LTLWithTriangulation)
  endif()


endif(OMPL_BUILD_DEMOS)

file(GLOB OMPL_DEMO_CXX_FILES "*.cpp")
file(GLOB OMPL_DEMO_PY_FILES "*.py")
install(FILES ${OMPL_DEMO_CXX_FILES} ${OMPL_DEMO_PY_FILES}
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)
install(DIRECTORY Koules
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)
