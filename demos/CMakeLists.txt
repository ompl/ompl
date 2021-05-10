option(OMPL_BUILD_DEMOS "Build OMPL demos" ON)
add_feature_info(OMPL_BUILD_DEMOS "${OMPL_BUILD_DEMOS}" "Whether to build the OMPL demo programs.")

if (OMPL_BUILD_DEMOS)
    file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/tests/resources" TEST_RESOURCES_DIR)
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
    add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

    macro(add_ompl_demo demo_name)
        add_executable(${ARGV})
        target_link_libraries(${demo_name}
            ompl
            ${Boost_FILESYSTEM_LIBRARY}
            ${Boost_SYSTEM_LIBRARY}
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
    add_ompl_demo(demo_ThunderLightning ThunderLightning.cpp)

    add_ompl_demo(demo_OptimalPlanning OptimalPlanning.cpp)
    add_ompl_demo(demo_PlannerProgressProperties PlannerProgressProperties.cpp)
    add_ompl_demo(demo_CForestCircleGridBenchmark CForestCircleGridBenchmark.cpp)

    add_ompl_demo(demo_Diagonal Diagonal.cpp)

    add_ompl_demo(demo_VectorFieldConservative VFRRT/VectorFieldConservative.cpp)
    add_ompl_demo(demo_VectorFieldNonconservative VFRRT/VectorFieldNonconservative.cpp)

    add_ompl_demo(demo_ConstrainedPlanningSphere constraint/ConstrainedPlanningSphere.cpp)
    add_ompl_demo(demo_ConstrainedPlanningTorus constraint/ConstrainedPlanningTorus.cpp)
    add_ompl_demo(demo_ConstrainedPlanningImplicitChain constraint/ConstrainedPlanningImplicitChain.cpp)
    add_ompl_demo(demo_ConstrainedPlanningImplicitParallel constraint/ConstrainedPlanningImplicitParallel.cpp)
    add_ompl_demo(demo_ConstrainedPlanningKinematicChain constraint/ConstrainedPlanningKinematicChain.cpp)

    add_ompl_demo(demo_QuotientSpacePlanningRigidBody2D quotientspace/QuotientSpacePlanningRigidBody2D.cpp)
    add_ompl_demo(demo_QuotientSpacePlanningRigidBody3D quotientspace/QuotientSpacePlanningRigidBody3D.cpp)
    add_ompl_demo(demo_QuotientSpacePlanningKinematicChain quotientspace/QuotientSpacePlanningKinematicChain.cpp)
    add_ompl_demo(demo_QuotientSpacePlanningHyperCube quotientspace/QuotientSpacePlanningHyperCube.cpp)
    add_ompl_demo(demo_QuotientSpacePlanningHyperCubeBenchmark quotientspace/QuotientSpacePlanningHyperCubeBenchmark.cpp)

    find_package(yaml-cpp)
    set_package_properties(yaml-cpp PROPERTIES
        URL "https://github.com/jbeder/yaml-cpp"
        PURPOSE "Used for parsing input data for the PlanarManipulator demo.")
    if (yaml-cpp_FOUND)
        add_ompl_demo(demo_PlanarManipulator
            PlanarManipulator/PlanarManipulatorDemo.cpp
            PlanarManipulator/PlanarManipulator.cpp
            PlanarManipulator/PolyWorld.cpp
        )
        target_include_directories(demo_PlanarManipulator PUBLIC ${YAMLCPP_INCLUDE_DIRS})
        target_link_libraries(demo_PlanarManipulator ${YAMLCPP_LIBRARIES})
    endif()

    if (OMPL_EXTENSION_ODE)
        add_ompl_demo(demo_OpenDERigidBodyPlanning OpenDERigidBodyPlanning.cpp)
    endif()

    if (OMPL_EXTENSION_TRIANGLE)
        add_ompl_demo(demo_TriangulationDemo TriangulationDemo.cpp)
        add_ompl_demo(demo_LTLWithTriangulation LTLWithTriangulation)
    endif()
endif(OMPL_BUILD_DEMOS)

file(GLOB OMPL_DEMO_PY_FILES "*.py")
install_python(PROGRAMS ${OMPL_DEMO_PY_FILES}
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)
file(GLOB OMPL_DEMO_CXX_FILES "*.cpp")
install(FILES ${OMPL_DEMO_CXX_FILES}
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)
install(DIRECTORY Koules
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)
install(DIRECTORY VFRRT
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)
