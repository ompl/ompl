#pragma once

#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <array>
#include <map>
#include <stdexcept>
#include <algorithm>

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
    static constexpr size_t dimension_ = Robot::dimension;
    
    virtual ~RobotConfig() = default;
    
    /**
     * @brief Get joint limits for the robot
     * @return Vector of (min, max) pairs for each joint
     */
    virtual auto get_joint_limits() const -> std::vector<std::pair<double, double>> = 0;
    
    /**
     * @brief Get a default start configuration for demonstrations
     * @return Start configuration as array of floats
     */
    virtual auto get_start_configuration_array() const -> std::array<float, dimension_> = 0;
    
    /**
     * @brief Get a default goal configuration for demonstrations  
     * @return Goal configuration as array of floats
     */
    virtual auto get_goal_configuration_array() const -> std::array<float, dimension_> = 0;
    
    /**
     * @brief Get robot name for display purposes
     * @return Human-readable robot name
     */
    virtual auto get_robot_name() const -> std::string = 0;

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
    virtual auto create_environment() -> vamp::collision::Environment<float> = 0;
    
    /**
     * @brief Get environment name for display purposes
     * @return Human-readable environment name  
     */
    virtual auto get_environment_name() const -> std::string = 0;
    
    /**
     * @brief Get environment description
     * @return Description of what this environment contains
     */
    virtual auto get_description() const -> std::string = 0;
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
    
    // Constructor with essential parameters
    PlanningConfig(double plan_time, double simp_time, bool optimize, 
                  const std::string& planner, bool write_path_to_file = false)
        : planning_time(plan_time)
        , simplification_time(simp_time)
        , optimize_path(optimize)
        , planner_name(planner)
        , write_path(write_path_to_file) {
    }
    
    // Constructor with all parameters including planner-specific options
    PlanningConfig(double plan_time, double simp_time, bool optimize, 
                  const std::string& planner, bool write_path_to_file, 
                  const std::map<std::string, std::string>& params)
        : planning_time(plan_time)
        , simplification_time(simp_time)
        , optimize_path(optimize)
        , planner_name(planner)
        , write_path(write_path_to_file)
        , planner_parameters(params) {
    }
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
 * @brief Configuration structure for obstacles
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
 * @brief Planning configuration with explicit requirements
 */
struct PlanningConfiguration {
    std::string robot_name;
    std::string description;
    bool save_path = false;
    
    // Planning configuration - MUST be explicitly set
    PlanningConfig planning;
    
    // Start/goal configurations - MUST be provided
    std::vector<float> start_config;
    std::vector<float> goal_config;
    
    // Obstacle configurations - MUST be provided
    std::vector<ObstacleConfig> obstacles;
    
    // Visualization configuration
    struct VisualizationConfig {
        std::string urdf_path;              // Path to robot URDF for visualization
        int expected_joints = -1;           // Expected number of joints (-1 = auto-detect)
        std::array<float, 3> base_position = {0.0f, 0.0f, 0.0f};  // Robot base position
        std::array<float, 3> base_orientation = {0.0f, 0.0f, 0.0f}; // Robot base orientation (euler)
        bool use_fixed_base = true;         // Whether robot base is fixed
        std::string description;            // Visualization description
    } visualization;
    
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
 * @brief Planning result that extends PlanningResult
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

/**
 * @brief Extract joint limits from VAMP robot definitions
 */
template<typename Robot>
auto get_joint_limits_from_vamp() -> std::vector<std::pair<double, double>> {
    std::vector<std::pair<double, double>> limits;
    limits.reserve(Robot::dimension);
    
    // VAMP stores limits in s_a (lower) and s_m (range) arrays
    for (size_t i = 0; i < Robot::dimension; ++i) {
        double lower = Robot::s_a[i];
        double upper = Robot::s_a[i] + Robot::s_m[i];
        limits.emplace_back(lower, upper);
    }
    
    return limits;
}

/**
 * @brief Robot configuration implementation
 */
template<typename Robot>
class RobotConfiguration : public RobotConfig<Robot> {
private:
    std::vector<float> start_config_;
    std::vector<float> goal_config_;
    std::string robot_name_;
    
    /**
     * @brief Validate configuration dimensions against robot requirements
     */
    void validate_dimensions() const {
        if (start_config_.size() != Robot::dimension) {
            throw std::invalid_argument(
                "Start configuration dimension (" + std::to_string(start_config_.size()) + 
                ") does not match robot dimension (" + std::to_string(Robot::dimension) + 
                ") for " + robot_name_);
        }
        
        if (goal_config_.size() != Robot::dimension) {
            throw std::invalid_argument(
                "Goal configuration dimension (" + std::to_string(goal_config_.size()) + 
                ") does not match robot dimension (" + std::to_string(Robot::dimension) + 
                ") for " + robot_name_);
        }
    }
    
    /**
     * @brief Validate configuration values against joint limits
     */
    void validate_joint_limits() const {
        auto limits = get_joint_limits_from_vamp<Robot>();
        
        auto check_config_limits = [&](const std::vector<float>& config, 
                                      const std::string& config_type) {
            for (size_t i = 0; i < Robot::dimension; ++i) {
                if (config[i] < limits[i].first || config[i] > limits[i].second) {
                    throw std::out_of_range(
                        config_type + " joint " + std::to_string(i) + 
                        " (" + std::to_string(config[i]) + ") outside limits [" + 
                        std::to_string(limits[i].first) + "," + 
                        std::to_string(limits[i].second) + "] for robot " + robot_name_);
                }
            }
        };
        
        check_config_limits(start_config_, "Start");
        check_config_limits(goal_config_, "Goal");
    }
    
    /**
     * @brief Convert vector to array safely
     */
    auto vector_to_array(const std::vector<float>& vec) const -> std::array<float, Robot::dimension> {
        std::array<float, Robot::dimension> result;
        std::copy(vec.begin(), vec.end(), result.begin());
        return result;
    }
    
public:
    RobotConfiguration(const std::string& robot_name, 
                      std::vector<float> start, 
                      std::vector<float> goal)
        : start_config_(std::move(start))
        , goal_config_(std::move(goal))
        , robot_name_(robot_name) {
        
        // TODO:
        // We can't use VampUtils here due to circular dependencies.
        // VampUtils includes this header, so we keep validation logic here
        // but this could be refactored in the future with better header organization.
        
        validate_dimensions();
        validate_joint_limits();
    }
    
    // Modern interface methods
    auto get_joint_limits() const -> std::vector<std::pair<double, double>> override {
        return get_joint_limits_from_vamp<Robot>();
    }
    
    auto get_start_configuration_array() const -> std::array<float, Robot::dimension> override {
        return vector_to_array(start_config_);
    }
    
    auto get_goal_configuration_array() const -> std::array<float, Robot::dimension> override {
        return vector_to_array(goal_config_);
    }
    
    auto get_robot_name() const -> std::string override {
        return robot_name_;
    }
};

} // namespace vamp_ompl