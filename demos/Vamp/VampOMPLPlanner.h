#pragma once

#include "VampOMPLInterfaces.h"
#include "OMPLPlanningContext.h"
#include <memory>
#include <iostream>

namespace vamp_ompl {

/**
 * @brief Main VAMP-OMPL integration planner
 * 
 * This class acts as a thin coordination layer that brings together:
 * - Robot configuration (joint limits, default poses)
 * - Environment factory (collision environment creation)
 * - OMPL planning context (space setup, planning execution)
 * 
 * The class follows the single responsibility principle and delegates
 * specific tasks to specialized components.
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
        
        return ompl_context_.plan(config);
    }
    
    /**
     * @brief Get robot configuration (for inspection or modification)
     * @return Reference to robot configuration
     */
    const RobotConfig<Robot>& getRobotConfig() const
    {
        return *robot_config_;
    }
    
    /**
     * @brief Get environment factory (for inspection)
     * @return Reference to environment factory
     */
    const EnvironmentFactory& getEnvironmentFactory() const
    {
        return *env_factory_;
    }
    
    /**
     * @brief Get OMPL planning context (for advanced usage)
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