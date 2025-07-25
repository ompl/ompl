#pragma once

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include <vamp/collision/environment.hh>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Path.h>

namespace vamp_ompl {

namespace ob = ompl::base;

/**
 * @brief Interface for robot configuration that provides robot-specific parameters
 * 
 * This interface abstracts robot-specific details like joint limits, dimensions,
 * and default configurations. It allows easy extension to new robot types.
 */
template<typename Robot>
class RobotConfig {
public:
    using Configuration = typename Robot::Configuration;
    static constexpr size_t dimension = Robot::dimension;
    
    virtual ~RobotConfig() = default;
    
    /**
     * @brief Get joint limits for the robot
     * @return Vector of (min, max) pairs for each joint
     */
    virtual std::vector<std::pair<double, double>> getJointLimits() const = 0;
    
    /**
     * @brief Get a default start configuration for demonstrations
     * @return Start configuration as array of floats
     */
    virtual std::array<float, dimension> getStartConfigurationArray() const = 0;
    
    /**
     * @brief Get a default goal configuration for demonstrations  
     * @return Goal configuration as array of floats
     */
    virtual std::array<float, dimension> getGoalConfigurationArray() const = 0;
    
    /**
     * @brief Get robot name for display purposes
     * @return Human-readable robot name
     */
    virtual std::string getRobotName() const = 0;
};

/**
 * @brief Interface for environment factory that creates collision environments
 * 
 * This interface allows easy creation of different test environments
 * without coupling to specific environment implementations.
 */
class EnvironmentFactory {
public:
    virtual ~EnvironmentFactory() = default;
    
    /**
     * @brief Create a collision environment
     * @return VAMP collision environment
     */
    virtual vamp::collision::Environment<float> createEnvironment() = 0;
    
    /**
     * @brief Get environment name for display purposes
     * @return Human-readable environment name  
     */
    virtual std::string getEnvironmentName() const = 0;
    
    /**
     * @brief Get environment description
     * @return Description of what this environment contains
     */
    virtual std::string getDescription() const = 0;
};

/**
 * @brief Planning configuration that holds timing and optimization settings
 */
struct PlanningConfig {
    double planning_time = 1.0;        ///< Maximum planning time in seconds
    double simplification_time = 0.5;  ///< Time for path simplification
    bool optimize_path = false;         ///< Whether to optimize for path cost
    std::string planner_name = "BIT*"; ///< Name of planner to use
    
    PlanningConfig() = default;
    PlanningConfig(double plan_time, double simp_time, bool optimize, const std::string& planner)
        : planning_time(plan_time), simplification_time(simp_time), 
          optimize_path(optimize), planner_name(planner) {}
};

/**
 * @brief Results from a planning attempt
 */
struct PlanningResult {
    bool success = false;                    ///< Whether planning succeeded
    double planning_time_us = 0.0;          ///< Actual planning time in microseconds
    double simplification_time_us = 0.0;    ///< Actual simplification time in microseconds
    double initial_cost = 0.0;              ///< Initial path cost before simplification
    double final_cost = 0.0;                ///< Final path cost after simplification
    size_t path_length = 0;                 ///< Number of states in final path
    std::string error_message;              ///< Error message if planning failed
    ob::PathPtr solution_path;              ///< The solution path (if successful)
};

} // namespace vamp_ompl