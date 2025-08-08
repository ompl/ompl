#pragma once

#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <array>
#include <map>

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
    bool write_path = false;           ///< Whether to write solution path to file
    std::map<std::string, std::string> planner_parameters; ///< Planner-specific parameters (key-value pairs)
    
    PlanningConfig() = default;
    PlanningConfig(double plan_time, double simp_time, bool optimize, const std::string& planner, bool write_path_to_file = false)
        : planning_time(plan_time), simplification_time(simp_time), 
          optimize_path(optimize), planner_name(planner), write_path(write_path_to_file) {}
    
    PlanningConfig(double plan_time, double simp_time, bool optimize, const std::string& planner, 
                   bool write_path_to_file, const std::map<std::string, std::string>& params)
        : planning_time(plan_time), simplification_time(simp_time), 
          optimize_path(optimize), planner_name(planner), write_path(write_path_to_file), planner_parameters(params) {}
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
    std::string solution_file_path;         ///< Path to written solution file (if write_path enabled)
};

/**
 * @brief Configuration structure for obstacles (Single Responsibility)
 */
struct ObstacleConfig {
    std::string type;           // "sphere", "cuboid", "capsule", "pointcloud"
    std::string name;           // Optional name for the obstacle
    std::array<float, 3> position;
    std::array<float, 3> orientation_euler_xyz = {0.0f, 0.0f, 0.0f}; // For cuboids and capsules
    float radius = 0.1f;        // For spheres and capsules
    std::array<float, 3> half_extents = {0.1f, 0.1f, 0.1f}; // For cuboids
    float length = 0.2f;        // For capsules
    
    // Pointcloud-specific parameters
    std::string pointcloud_file;  // Path to pointcloud file (.xyz, .ply, .pcd)
    float point_radius = 0.0025f; // Radius for each point in pointcloud
    
    ObstacleConfig() = default;
    
    ObstacleConfig(const std::string& obstacle_type, 
                   const std::array<float, 3>& pos,
                   float r = 0.1f)
        : type(obstacle_type), position(pos), radius(r) {}
        
    ObstacleConfig(const std::string& obstacle_type,
                   const std::array<float, 3>& pos,
                   const std::array<float, 3>& half_ext)
        : type(obstacle_type), position(pos), half_extents(half_ext) {}
        
    // Pointcloud constructor
    ObstacleConfig(const std::string& file_path, float pt_radius = 0.0025f)
        : type("pointcloud"), pointcloud_file(file_path), point_radius(pt_radius) {}
};

/**
 * @brief Planning configuration with explicit requirements (No Defaults)
 */
struct PlanningConfiguration {
    std::string robot_name;
    std::string description;
    bool save_path = false;
    
    // Planning configuration - MUST be explicitly set
    PlanningConfig planning;
    
    // Start/goal configurations - MUST be provided (no defaults)
    std::vector<float> start_config;
    std::vector<float> goal_config;
    
    // Obstacle configurations - MUST be provided (no named environments)
    std::vector<ObstacleConfig> obstacles;
    
    // Validation helper
    bool isValid() const {
        return !robot_name.empty() && 
               !start_config.empty() && 
               !goal_config.empty() &&
               !planning.planner_name.empty() &&
               planning.planning_time > 0.0;
    }
    
    std::string getValidationErrors() const {
        std::string errors;
        if (robot_name.empty()) errors += "Robot name not specified. ";
        if (start_config.empty()) errors += "Start configuration not provided. ";
        if (goal_config.empty()) errors += "Goal configuration not provided. ";
        if (planning.planner_name.empty()) errors += "Planner name not specified. ";
        if (planning.planning_time <= 0.0) errors += "Invalid planning time. ";
        return errors;
    }
};

/**
 * @brief Planning result that extends PlanningResult (Single Responsibility Principle)
 */
struct MotionPlanningResult {
    PlanningResult planning_result;
    std::string solution_file_path;
    std::string robot_name; // Store robot name for visualization
    
    // Convenience accessors
    bool success() const { return planning_result.success; }
    double planning_time_us() const { return planning_result.planning_time_us; }
    double final_cost() const { return planning_result.final_cost; }
    size_t path_length() const { return planning_result.path_length; }
    const std::string& error_message() const { return planning_result.error_message; }
};

} // namespace vamp_ompl