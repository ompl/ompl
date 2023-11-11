option(OMPL_BUILD_DEMOS "Build OMPL demos" ON)
add_feature_info(OMPL_BUILD_DEMOS "${OMPL_BUILD_DEMOS}" "Whether to build the OMPL demo programs.")

if (OMPL_BUILD_DEMOS)
    file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/tests/resources" TEST_RESOURCES_DIR)
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
    add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

    macro(add_ompl_demo demo_name)
        add_executable(${ARGV})
        target_link_libraries(${demo_name} PRIVATE
            ompl::ompl
            Eigen3::Eigen
            Boost::filesystem
            Boost::system
            Boost::program_options)
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
    
    add_ompl_demo(demo_MultiLevelPlanningRigidBody2D multilevel/MultiLevelPlanningRigidBody2D.cpp)
    add_ompl_demo(demo_MultiLevelPlanningRigidBody3D multilevel/MultiLevelPlanningRigidBody3D.cpp)
    add_ompl_demo(demo_MultiLevelPlanningKinematicChain multilevel/MultiLevelPlanningKinematicChain.cpp)
    add_ompl_demo(demo_MultiLevelPlanningHyperCube multilevel/MultiLevelPlanningHyperCube.cpp)
    add_ompl_demo(demo_MultiLevelPlanningHyperCubeBenchmark multilevel/MultiLevelPlanningHyperCubeBenchmark.cpp)

    add_ompl_demo(demo_SpaceTimePlanning SpaceTimePlanning.cpp)

    find_package(yaml-cpp CONFIG)
    set_package_properties(yaml-cpp PROPERTIES
        URL "https://github.com/jbeder/yaml-cpp"
        PURPOSE "Used for parsing input data for the PlanarManipulator demo.")
    if (yaml-cpp_FOUND)
        # yaml-cpp might not come with alias or namespaced target names, so add it here.
        # https://github.com/jbeder/yaml-cpp/issues/1025
        # https://github.com/jbeder/yaml-cpp/pull/1196
        if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
            # Aliasing local imported targets is only supported in CMake >=3.18.
            # Promoting yaml-cpp to the global scope here prevents errors on
            # older CMake versions.
            # (see: https://cmake.org/cmake/help/latest/command/add_library.html#alias-libraries)
            set_target_properties(yaml-cpp PROPERTIES IMPORTED_GLOBAL TRUE)

            add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
        endif()

        add_ompl_demo(demo_PlanarManipulator
            PlanarManipulator/PlanarManipulatorDemo.cpp
            PlanarManipulator/PlanarManipulator.cpp
            PlanarManipulator/PolyWorld.cpp
        )
        target_link_libraries(demo_PlanarManipulator PRIVATE yaml-cpp::yaml-cpp)

        add_ompl_demo(demo_MultiLevelPlanarManipulator
            multilevel/MultiLevelPlanarManipulatorDemo.cpp
            PlanarManipulator/PlanarManipulator.cpp
            PlanarManipulator/PolyWorld.cpp
        )
        target_link_libraries(demo_MultiLevelPlanarManipulator PRIVATE yaml-cpp::yaml-cpp)
    endif()

    if (OMPL_EXTENSION_TRIANGLE)
        add_ompl_demo(demo_TriangulationDemo TriangulationDemo.cpp)
        add_ompl_demo(demo_LTLWithTriangulation LTLWithTriangulation.cpp)
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
