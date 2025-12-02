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
            Boost::program_options)
    endmacro(add_ompl_demo)


    add_ompl_demo(demo_SSTPinballPlanning SSTPinballPlanning.cpp)
    add_ompl_demo(demo_PinballPlanning PinballPlanning.cpp)
    add_ompl_demo(demo_BouncingBallPlanning BouncingBallPlanning.cpp)    
    add_ompl_demo(demo_RigidBodyPlanning RigidBodyPlanning.cpp)
    add_ompl_demo(demo_RigidBodyPlanningWithIK RigidBodyPlanningWithIK.cpp)
    add_ompl_demo(demo_RigidBodyPlanningWithControls RigidBodyPlanningWithControls.cpp)
    add_ompl_demo(demo_RigidBodyPlanningWithIntegrationAndControls RigidBodyPlanningWithIntegrationAndControls.cpp)
    add_ompl_demo(demo_RigidBodyPlanningWithODESolverAndControls RigidBodyPlanningWithODESolverAndControls.cpp)
    add_ompl_demo(demo_StateSampling StateSampling.cpp)
    add_ompl_demo(demo_GeometricCarPlanning GeometricCarPlanning.cpp)
    add_ompl_demo(demo_DubinsAirplane DubinsAirplane.cpp)
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

    # yaml-cpp is now a required dependency and target alias is handled in main CMakeLists.txt

        add_ompl_demo(demo_MulticopterPlanning 
                    multicopter/MulticopterPlanning.cpp
                    multicopter/Quartic.cpp)

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

    if (OMPL_EXTENSION_TRIANGLE)
        add_ompl_demo(demo_TriangulationDemo TriangulationDemo.cpp)
        add_ompl_demo(demo_LTLWithTriangulation LTLWithTriangulation.cpp)
    endif()

    # Add VAMP demo only if VAMP is available and properly configured
    if (OMPL_HAVE_VAMP)
        # Build VAMP demo directly using VAMP C++ library and OMPL
        add_ompl_demo(demo_Vamp Vamp/demos/VampDemo.cpp)
        
        # Add include directories for VAMP headers
        target_include_directories(demo_Vamp PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/core
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/utilities
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/benchmarking
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/examples)
        
        # Link against VAMP C++ library for all demos
        if(TARGET vamp_cpp)
            target_link_libraries(demo_Vamp PRIVATE vamp_cpp)
            message(STATUS "VAMP demos linked against vamp_cpp library")
        else()
            message(WARNING "VAMP vamp_cpp target not found - demos may not build correctly")
        endif()
        
        # Link against yaml-cpp for configuration parsing (now required)
        target_link_libraries(demo_Vamp PRIVATE yaml-cpp::yaml-cpp)
        message(STATUS "VAMP demos linked against yaml-cpp library")

        # Copy configuration files to build directory for testing
        configure_file(
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/config/panda_demo.yaml
            ${CMAKE_CURRENT_BINARY_DIR}/panda_demo.yaml
            COPYONLY)
        
        # Copy custom robot configuration files to build directory for testing
        configure_file(
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/config/planar_arm_2dof_demo.yaml
            ${CMAKE_CURRENT_BINARY_DIR}/planar_arm_2dof_demo.yaml
            COPYONLY)
        
        # Copy custom robot URDF file to build directory for visualization
        configure_file(
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/config/planar_arm_2dof.urdf
            ${CMAKE_CURRENT_BINARY_DIR}/planar_arm_2dof.urdf
            COPYONLY)

        # Copy benchmark configuration files to build directory for testing
        configure_file(
            ${CMAKE_CURRENT_SOURCE_DIR}/Vamp/config/panda_benchmark.yaml
            ${CMAKE_CURRENT_BINARY_DIR}/panda_benchmark.yaml
            COPYONLY)
        
        message(STATUS "VAMP demo (demo_Vamp) will be built using VAMP C++ library and OMPL")
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
install(DIRECTORY constraint Koules multilevel PlanarManipulator VFRRT
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT ompl)
