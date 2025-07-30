#pragma once

#include "VampOMPLInterfaces.h"
#include "OMPLPlanningContext.h"
#include <memory>
#include <iostream>

namespace vamp_ompl {

/**
 * @brief Main VAMP-OMPL integration planner implementing the Facade pattern
 * 
 * This class acts as a unified interface that coordinates between the VAMP collision
 * detection system and OMPL's motion planning algorithms. It implements the Facade
 * design pattern to hide the complexity of integrating these two systems.
 * 
 * Key Responsibilities:
 * - Configuration management: Maintains robot and environment configurations
 * - Lifecycle management: Handles initialization, planning, and cleanup phases
 * - Performance optimization: Leverages VAMP's vectorized collision detection
 * - Result processing: Handles path writing, timing, and result formatting
 * 
 * Architecture Benefits:
 * - Single point of integration between VAMP and OMPL
 * - Type-safe template-based robot support
 * - Minimal coupling between subsystems
 * - Consistent error handling and reporting
 * - Extensible to new robot types and environments
 * 
 * The class follows the single responsibility principle and delegates
 * specific tasks to specialized components.
 * This class acts as a unified interface that coordinates between the VAMP collision
 * detection system and OMPL's motion planning algorithms. It implements the Facade
 * design pattern to hide the complexity of integrating these two systems.
 * 
 * @tparam Robot VAMP robot type (e.g., vamp::robots::Panda)
 */
template<typename Robot>
class VampOMPLPlanner {
public:
    using Configuration = typename Robot::Configuration;
    static constexpr std::size_t dimension = Robot::dimension;
    static constexpr std::size_t rake = vamp::FloatVectorWidth;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

private:
    std::unique_ptr<RobotConfig<Robot>> robot_config_;
    std::unique_ptr<EnvironmentFactory> env_factory_;
    OMPLPlanningContext<Robot> ompl_context_;
    EnvironmentVector environment_;
    bool initialized_;

public:
    /**
     * @brief Constructor taking robot and environment configuration
     * @param robot_config Robot configuration providing limits and poses
     * @param env_factory Factory for creating collision environments
     */
    VampOMPLPlanner(std::unique_ptr<RobotConfig<Robot>> robot_config,
                    std::unique_ptr<EnvironmentFactory> env_factory)
        : robot_config_(std::move(robot_config)), 
          env_factory_(std::move(env_factory)),
          initialized_(false)
    {
        if (!robot_config_) {
            throw std::invalid_argument("Robot configuration cannot be null");
        }
        if (!env_factory_) {
            throw std::invalid_argument("Environment factory cannot be null");
        }
    }

    /**
     * @brief Initialize the planner (create environment, setup OMPL)
     * This must be called before planning
     */
    void initialize()
    {
        std::cout << "Initializing VAMP-OMPL Planner..." << std::endl;
        std::cout << "Robot: " << robot_config_->getRobotName() << std::endl;
        std::cout << "Environment: " << env_factory_->getEnvironmentName() << std::endl;
        std::cout << "Description: " << env_factory_->getDescription() << std::endl;
        
        // Create collision environment
        auto env_input = env_factory_->createEnvironment();
        environment_ = EnvironmentVector(env_input);
        
        // Setup OMPL state space and validators
        ompl_context_.setupStateSpace(*robot_config_, environment_);
        
        // Setup default problem (start and goal from robot config)
        auto start_config = robot_config_->getStartConfigurationArray();
        auto goal_config = robot_config_->getGoalConfigurationArray();
        ompl_context_.setProblem(start_config, goal_config);
        
        initialized_ = true;
        std::cout << "✓ Initialization complete" << std::endl;
    }
    
    /**
     * @brief Plan with default robot start/goal configurations
     * @param config Planning configuration (time limits, planner type, etc.)
     * @return Planning results
     */
    PlanningResult plan(const PlanningConfig &config = PlanningConfig())
    {
        if (!initialized_) {
            throw std::runtime_error("Planner not initialized. Call initialize() first.");
        }
        
        return ompl_context_.plan(config);
    }
    
    /**
     * @brief Plan with custom start and goal configurations
     * @param start_config Custom start configuration as array
     * @param goal_config Custom goal configuration as array 
     * @param config Planning configuration
     * @return Planning results
     */
    PlanningResult plan(const std::array<float, dimension> &start_config, 
                       const std::array<float, dimension> &goal_config,
                       const PlanningConfig &config = PlanningConfig())
    {
        if (!initialized_) {
            throw std::runtime_error("Planner not initialized. Call initialize() first.");
        }
        
        // Set custom problem
        ompl_context_.setProblem(start_config, goal_config);
        
        auto result = ompl_context_.plan(config);
        
        // Write solution path to file if requested and planning succeeded
        if (config.write_path && result.success && result.solution_path) {
            try {
                writeSolutionPath(result, config.planner_name);
            } catch (const std::exception& e) {
                std::cerr << "Warning: Failed to write solution path: " << e.what() << std::endl;
            }
        }
        
        return result;
    }

    /**
     * @brief Write the interpolated solution path to a text file
     * @param result Planning result containing the solution path
     * @param planner_name Name of the planner used (for filename)
     * @param num_waypoints Number of waypoints to interpolate (default: 100)
     * @param output_dir Output directory path (default: tries demos/Vamp/, falls back to current dir)
     */
    void writeSolutionPath(const PlanningResult& result, 
                          const std::string& planner_name,
                          size_t num_waypoints = 100,
                          const std::string& output_dir = "") const
    {
        if (!result.success || !result.solution_path) {
            throw std::runtime_error("No valid solution path to write");
        }

        // Cast to geometric path
        auto path_geometric = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(result.solution_path);
        if (!path_geometric) {
            throw std::runtime_error("Solution path is not a geometric path");
        }

        // Interpolate the path to get evenly spaced waypoints
        path_geometric->interpolate(num_waypoints);

        // Determine output directory
        std::string target_dir = output_dir;
        if (target_dir.empty()) {
            // Try to use demos/Vamp/ directory first
            if (std::filesystem::exists("demos/Vamp/")) {
                target_dir = "demos/Vamp/";
            } else if (std::filesystem::exists("../demos/Vamp/")) {
                target_dir = "../demos/Vamp/";
            } else if (std::filesystem::exists("../../demos/Vamp/")) {
                target_dir = "../../demos/Vamp/";
            } else {
                // Fall back to current directory
                target_dir = "./";
            }
        }

        // Ensure directory exists
        std::filesystem::create_directories(target_dir);

        // Generate filename with robot, environment, and planner info
        std::string filename = target_dir + "solution_path_" + 
                              robot_config_->getRobotName() + "_" +
                              env_factory_->getEnvironmentName() + "_" + 
                              planner_name + ".txt";

        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        // Write header
        file << "# Solution Path for " << robot_config_->getRobotName() 
             << " in " << env_factory_->getEnvironmentName() 
             << " environment using " << planner_name << " planner\n";
        file << "# Format: Each row contains joint values for one waypoint\n";
        file << "# Number of joints: " << dimension << "\n";
        file << "# Number of waypoints: " << path_geometric->getStateCount() << "\n";
        file << "# Path cost: " << result.final_cost << "\n";
        file << "#\n";

        // Write column headers
        file << "# ";
        for (size_t i = 0; i < dimension; ++i) {
            file << "joint_" << i;
            if (i < dimension - 1) file << "\t";
        }
        file << "\n";

        // Write waypoints
        for (size_t i = 0; i < path_geometric->getStateCount(); ++i) {
            const auto* state = path_geometric->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            
            for (size_t j = 0; j < dimension; ++j) {
                file << std::fixed << std::setprecision(6) << (*state)[j];
                if (j < dimension - 1) file << "\t";
            }
            file << "\n";
        }

        file.close();
        std::cout << "✓ Solution path written to: " << filename << std::endl;
        std::cout << "  - Waypoints: " << path_geometric->getStateCount() << std::endl;
        std::cout << "  - Path cost: " << result.final_cost << std::endl;
    }
    
    /**
     * @brief Get robot configuration
     * @return Reference to robot configuration
     */
    const RobotConfig<Robot>& getRobotConfig() const
    {
        return *robot_config_;
    }
    
    /**
     * @brief Get environment factory
     * @return Reference to environment factory
     */
    const EnvironmentFactory& getEnvironmentFactory() const
    {
        return *env_factory_;
    }
    
    /**
     * @brief Get OMPL planning context
     * @return Reference to OMPL planning context
     */
    const OMPLPlanningContext<Robot>& getOMPLContext() const
    {
        return ompl_context_;
    }
    
    /**
     * @brief Check if planner is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const
    {
        return initialized_;
    }
    
    /**
     * @brief Print configuration summary
     */
    void printConfiguration() const
    {
        std::cout << "\n=== VAMP-OMPL Planner Configuration ===" << std::endl;
        std::cout << "Robot: " << robot_config_->getRobotName() << std::endl;
        std::cout << "Dimensions: " << dimension << std::endl;
        std::cout << "Environment: " << env_factory_->getEnvironmentName() << std::endl;
        std::cout << "Description: " << env_factory_->getDescription() << std::endl;
        std::cout << "Initialized: " << (initialized_ ? "Yes" : "No") << std::endl;
        
        if (initialized_) {
            std::cout << "\nJoint Limits:" << std::endl;
            auto limits = robot_config_->getJointLimits();
            for (size_t i = 0; i < limits.size(); ++i) {
                std::cout << "  Joint " << i << ": [" << limits[i].first 
                         << ", " << limits[i].second << "]" << std::endl;
            }
        }
        std::cout << std::string(40, '=') << std::endl;
    }
};

/**
 * @brief Convenience function to create a VAMP-OMPL planner
 * @param robot_config Robot configuration
 * @param env_factory Environment factory
 * @return Unique pointer to configured planner
 */
template<typename Robot>
std::unique_ptr<VampOMPLPlanner<Robot>> createVampOMPLPlanner(
    std::unique_ptr<RobotConfig<Robot>> robot_config,
    std::unique_ptr<EnvironmentFactory> env_factory)
{
    return std::make_unique<VampOMPLPlanner<Robot>>(
        std::move(robot_config), std::move(env_factory));
}

} // namespace vamp_ompl