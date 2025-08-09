#pragma once

#include "VampOMPLInterfaces.h"
#include "OMPLPlanningContext.h"
#include "VampUtils.h"
#include <ompl/geometric/PathGeometric.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <filesystem>

namespace vamp_ompl {

/**
 * @brief Main VAMP-OMPL integration planner implementing the Facade pattern
 * 
 * This class acts as a unified interface that coordinates between the VAMP collision
 * detection system and OMPL's motion planning algorithms. It implements the Facade
 * design pattern to hide the complexity of integrating these two systems.
 * 
 *  Design Patterns:
 * 1. Facade Pattern: Provides a simplified interface to complex subsystem integration
 * 2. Template Method: Standardizes the planning workflow while allowing customization
 * 3. RAII (Resource Acquisition Is Initialization): Automatic resource management
 * 4. Dependency Injection: Accepts configured dependencies rather than creating them
 * 
 * Key Responsibilities:
 * - Configuration management: Maintains robot and environment configurations
 * - Lifecycle management: Handles initialization, planning, and cleanup phases
 * - Performance optimization: Leverages VAMP's vectorized collision detection
 * - Result processing: Handles path writing, timing, and result formatting
 * 
 * Integration Architecture Benefits:
 * - Single point of integration between VAMP and OMPL subsystems
 * - Type-safe template-based robot support with compile-time validation
 * - Minimal coupling between motion planning and collision detection
 * - Consistent error handling and comprehensive reporting
 * - Extensible to new robot types and environments without code modification
 * 
 * Performance Characteristics:
 * - Vectorized collision detection: SIMD speedup over scalar implementations
 * - Zero-copy configuration conversion: Direct memory mapping between systems
 * - Lazy initialization: Environment creation deferred until actually needed
 * - Efficient path writing: Leverages OMPL's optimized serialization
 * 
 * The class follows the single responsibility principle and delegates
 * specific tasks to specialized components, making it easy to understand
 * and maintain.
 * 
 * @tparam Robot VAMP robot type (e.g., vamp::robots::Panda)
 */
template<typename Robot>
class VampOMPLPlanner {
public:
    using RobotConfiguration = typename Robot::Configuration;
    static constexpr std::size_t robotDimension = Robot::dimension;
    static constexpr std::size_t simdLaneWidth = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simdLaneWidth>>;

private:
    std::unique_ptr<RobotConfig<Robot>> m_robotConfiguration;
    std::unique_ptr<EnvironmentFactory> m_environmentFactory;
    OMPLPlanningContext<Robot> m_planningContext;
    VectorizedEnvironment m_vectorizedEnvironment;
    bool m_isInitialized;

public:
    /**
     * @brief Constructor taking robot and environment configuration (Dependency Injection)
     * @param robotConfiguration Robot configuration providing limits and poses
     * @param environmentFactory Factory for creating collision environments
     * 
     *  Note: This constructor demonstrates dependency injection - rather than
     * creating its dependencies internally, the planner accepts them as parameters.
     * This design promotes testability, flexibility, and separation of concerns.
     */
    VampOMPLPlanner(std::unique_ptr<RobotConfig<Robot>> robotConfiguration,
                    std::unique_ptr<EnvironmentFactory> environmentFactory)
        : m_robotConfiguration(std::move(robotConfiguration)), 
          m_environmentFactory(std::move(environmentFactory)),
          m_isInitialized(false)
    {
        if (!m_robotConfiguration) {
            throw VampConfigurationError("Robot configuration cannot be null");
        }
        if (!m_environmentFactory) {
            throw VampConfigurationError("Environment factory cannot be null");
        }
    }

    /**
     * @brief Initialize the planner (create environment, setup OMPL)
     * This must be called before planning
     * 
     *  Note: This two-phase initialization pattern (constructor + initialize)
     * is common in complex systems where initialization can fail or is expensive.
     * It allows error handling and provides clear separation between object creation
     * and system setup.
     * 
     * Performance Note: Environment vectorization happens here, converting scalar
     * collision geometry to SIMD-optimized format for speedup.
     */
    void initialize()
    {   
        // Create collision environment
        auto scalarEnvironment = m_environmentFactory->createEnvironment();
        m_vectorizedEnvironment = VectorizedEnvironment(scalarEnvironment);
        
        // Setup OMPL state space and validators
        m_planningContext.setupStateSpace(*m_robotConfiguration, m_vectorizedEnvironment);
        
        // Setup default problem (start and goal from robot config)
        auto defaultStartConfiguration = m_robotConfiguration->getStartConfigurationArray();
        auto defaultGoalConfiguration = m_robotConfiguration->getGoalConfigurationArray();
        m_planningContext.setProblem(defaultStartConfiguration, defaultGoalConfiguration);
        
        m_isInitialized = true;
        std::cout << "Initialization complete" << std::endl;
    }
    
    /**
     * @brief Unified planning function - works with robot's default or custom start/goal
     * @param planningConfiguration Planning configuration (time limits, planner type, etc.)
     * @param customStartConfiguration Optional custom start configuration (if empty, uses robot's default)
     * @param customGoalConfiguration Optional custom goal configuration (if empty, uses robot's default)
     * @return Planning results with comprehensive timing and path information
     * 
     *  Note: This method demonstrates the Template Method pattern - it
     * defines a standard algorithm structure (validate → configure → plan → process)
     * while allowing customization through parameters.
     * 
     * Performance Note: Configuration validation happens at planning time rather than
     * construction time, allowing for dynamic reconfiguration without object recreation.
     */
    PlanningResult plan(const PlanningConfig &planningConfiguration = PlanningConfig(),
                       const std::array<float, robotDimension> &customStartConfiguration = {},
                       const std::array<float, robotDimension> &customGoalConfiguration = {})
    {
        if (!m_isInitialized) {
            throw VampConfigurationError("Planner not initialized. Call initialize() first.");
        }
        
        // Use custom start/goal if provided, otherwise use robot's defaults
        bool useCustomConfigurations = (customStartConfiguration != std::array<float, robotDimension>{});
        if (useCustomConfigurations) {
            m_planningContext.setProblem(customStartConfiguration, customGoalConfiguration);
        }
        
        auto planningResult = m_planningContext.plan(planningConfiguration);
        
        // Write solution path using OMPL's built-in functionality if requested
        if (planningConfiguration.write_path && planningResult.success && planningResult.solution_path) {
            planningResult.solution_file_path = writeOptimizedSolutionPath(planningResult, planningConfiguration.planner_name);
        }
        
        return planningResult;
    }

    /**
     * @brief Write solution path using OMPL's built-in printAsMatrix() functionality
     * 
     * File Format: The output format is compatible with standard robotics
     * visualization tools and can be easily imported into Python/MATLAB for analysis.
     */
    std::string writeOptimizedSolutionPath(const PlanningResult& planningResult, const std::string& plannerName, 
                                         const PlanningConfiguration::VisualizationConfig& visualizationConfig = {}) const
    {
        auto geometricPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(planningResult.solution_path);
        if (!planningResult.success || !geometricPath) return "";
        
        geometricPath->interpolate(constants::DEFAULT_PATH_WAYPOINTS); // Standard waypoint count
        
        // Find output directory and create filename
        auto findOutputDirectory = [](const std::vector<std::string>& candidatePaths) {
            for (const auto& pathCandidate : candidatePaths) {
                if (std::filesystem::exists(pathCandidate)) return pathCandidate;
            }
            return std::string("./");
        };
        
        std::string outputFilename = findOutputDirectory({"demos/Vamp/", "../demos/Vamp/", "../../demos/Vamp/"}) + 
                              "solution_path_" + m_robotConfiguration->getRobotName() + "_" + 
                              m_environmentFactory->getEnvironmentName() + "_" + plannerName + ".txt";
        
        std::ofstream outputFile(outputFilename);
        if (!outputFile) return "";
        
        // Write header using compact format
        outputFile << "# " << m_robotConfiguration->getRobotName() << " + " << m_environmentFactory->getEnvironmentName() 
             << " + " << plannerName << " (dim=" << robotDimension << ", waypoints=" 
             << geometricPath->getStateCount() << ", cost=" << planningResult.final_cost << ")\n";
        
        // Write visualization configuration if provided
        if (!visualizationConfig.urdf_path.empty()) {
            outputFile << "# VISUALIZATION CONFIG:\n";
            outputFile << "# robot_name: " << m_robotConfiguration->getRobotName() << "\n";
            outputFile << "# urdf_path: " << visualizationConfig.urdf_path << "\n";
            if (visualizationConfig.expected_joints > 0) {
                outputFile << "# expected_joints: " << visualizationConfig.expected_joints << "\n";
            }
            outputFile << "# base_position: [" << visualizationConfig.base_position[0] << ", " 
                      << visualizationConfig.base_position[1] << ", " << visualizationConfig.base_position[2] << "]\n";
            outputFile << "# base_orientation: [" << visualizationConfig.base_orientation[0] << ", " 
                      << visualizationConfig.base_orientation[1] << ", " << visualizationConfig.base_orientation[2] << "]\n";
            outputFile << "# use_fixed_base: " << (visualizationConfig.use_fixed_base ? "true" : "false") << "\n";
        }
        
        // Use OMPL's built-in path writing functionality
        geometricPath->printAsMatrix(outputFile);
        
        std::cout << " Path written: " << outputFilename << " (" << geometricPath->getStateCount() 
                  << " waypoints, cost=" << planningResult.final_cost << ")" << std::endl;
        return outputFilename;
    }
    
    /**
     * @brief Get robot configuration
     * @return Reference to robot configuration
     */
    const RobotConfig<Robot>& getRobotConfiguration() const
    {
        return *m_robotConfiguration;
    }
    
    /**
     * @brief Get environment factory
     * @return Reference to environment factory
     */
    const EnvironmentFactory& getEnvironmentFactory() const
    {
        return *m_environmentFactory;
    }
    
    /**
     * @brief Get OMPL planning context
     * @return Reference to OMPL planning context
     */
    const OMPLPlanningContext<Robot>& getPlanningContext() const
    {
        return m_planningContext;
    }
    
    /**
     * @brief Check if planner is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const
    {
        return m_isInitialized;
    }
    
    /**
     * @brief Print configuration summary (delegated to VampUtils)
     * 
     */
    void printConfiguration() const {
        VampUtils::printPlannerConfiguration<Robot>(*m_robotConfiguration, *m_environmentFactory, m_isInitialized);
    }
};

/**
 * @brief Convenience function to create a VAMP-OMPL planner (Factory Function)
 * @param robotConfiguration Robot configuration
 * @param environmentFactory Environment factory
 * @return Unique pointer to configured planner
 * 
 *  Note: This factory function encapsulates object creation logic,
 * making client code cleaner and providing a stable interface even if the
 * constructor signature changes in the future.
 */
template<typename Robot>
std::unique_ptr<VampOMPLPlanner<Robot>> createVampOMPLPlanner(
    std::unique_ptr<RobotConfig<Robot>> robotConfiguration,
    std::unique_ptr<EnvironmentFactory> environmentFactory)
{
    return std::make_unique<VampOMPLPlanner<Robot>>(
        std::move(robotConfiguration), std::move(environmentFactory));
}

} // namespace vamp_ompl