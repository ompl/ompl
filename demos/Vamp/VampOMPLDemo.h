/**
 * @file VampOMPLDemo.h
 * @brief VAMP-OMPL integration interface for vectorized motion planning
 * 
 * This file provides a clean demonstration of VAMP-OMPL integration,
 * showcasing vectorized motion planning with SIMD acceleration.
 */
#pragma once

#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include "OMPLPlanningContext.h"
#include "VampOMPLPlanner.h"
#include "VampUtils.h"

#include <memory>
#include <string>
#include <fstream>
#include <algorithm>
#include <sstream>

// VAMP robot includes
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>

namespace vamp_ompl {



/**
 * @brief Extract joint limits from VAMP robot definitions
 */
template<typename Robot>
std::vector<std::pair<double, double>> getJointLimitsFromVamp() {
    std::vector<std::pair<double, double>> limits;
    
    // VAMP stores limits in s_a (lower) and s_m (range) arrays
    for (size_t i = 0; i < Robot::dimension; ++i) {
        double lower = Robot::s_a[i];
        double upper = Robot::s_a[i] + Robot::s_m[i];
        limits.emplace_back(lower, upper);
    }
    
    return limits;
}

/**
 * @brief Robot configuration
 */
template<typename Robot>
class RobotConfiguration : public RobotConfig<Robot> {
private:
    std::vector<float> start_config_;
    std::vector<float> goal_config_;
    std::string robot_name_;
    
public:
    RobotConfiguration(const std::string& robot_name, 
                       const std::vector<float>& start,
                       const std::vector<float>& goal)
        : robot_name_(robot_name), start_config_(start), goal_config_(goal)
    {
        // Validate dimensions
        if (start_config_.size() != Robot::dimension) {
            throw VampConfigurationError("Start configuration dimension (" + 
                std::to_string(start_config_.size()) + ") does not match robot dimension (" + 
                std::to_string(Robot::dimension) + ") for " + robot_name);
        }
        
        if (goal_config_.size() != Robot::dimension) {
            throw VampConfigurationError("Goal configuration dimension (" + 
                std::to_string(goal_config_.size()) + ") does not match robot dimension (" + 
                std::to_string(Robot::dimension) + ") for " + robot_name);
        }
        
        // Validate joint limits
        auto limits = getJointLimitsFromVamp<Robot>();
        auto check_limits = [&](const std::vector<float>& config, const std::string& config_type) {
            for (size_t i = 0; i < Robot::dimension; ++i) {
                if (config[i] < limits[i].first || config[i] > limits[i].second) {
                    throw VampConfigurationError(config_type + " joint " + std::to_string(i) + 
                        " (" + std::to_string(config[i]) + ") outside limits [" + 
                        std::to_string(limits[i].first) + "," + std::to_string(limits[i].second) + "]");
                }
            }
        };
        check_limits(start_config_, "Start");
        check_limits(goal_config_, "Goal");
    }
    
    std::vector<std::pair<double, double>> getJointLimits() const override {
        return getJointLimitsFromVamp<Robot>();
    }
    
    std::array<float, Robot::dimension> getStartConfigurationArray() const override {
        return vectorToArray(start_config_);
    }
    
    std::array<float, Robot::dimension> getGoalConfigurationArray() const override {
        return vectorToArray(goal_config_);
    }
    
    std::string getRobotName() const override {
        return robot_name_;
    }

private:
    std::array<float, Robot::dimension> vectorToArray(const std::vector<float>& vec) const {
        std::array<float, Robot::dimension> result;
        std::copy(vec.begin(), vec.end(), result.begin());
        return result;
    }
};

/**
 * @brief Robot name mapping for type safety and conciseness
 */
template<typename Robot> constexpr const char* getRobotName();
template<> constexpr const char* getRobotName<vamp::robots::Panda>() { return "panda"; }
template<> constexpr const char* getRobotName<vamp::robots::UR5>() { return "ur5"; }
template<> constexpr const char* getRobotName<vamp::robots::Fetch>() { return "fetch"; }

/**
 * @brief Unified robot factory
 */
template<typename Robot>
std::unique_ptr<RobotConfig<Robot>> createRobotConfiguration(const std::string& robot_name,
                                                             const std::vector<float>& start,
                                                             const std::vector<float>& goal) {
    constexpr const char* expected_name = getRobotName<Robot>();
    if (robot_name != expected_name) {
        throw VampConfigurationError("Robot name must be '" + std::string(expected_name) + "' for " + std::string(expected_name) + " robot");
    }
    return std::make_unique<RobotConfiguration<Robot>>(expected_name, start, goal);
}

/**
 * @brief Environment factory for obstacle-based environments
 */
std::unique_ptr<EnvironmentFactory> createEnvironmentFactory(const std::vector<ObstacleConfig>& obstacles);

/**
 * @brief Main motion planning execution function (Dependency Inversion Principle)
 * Depends on abstractions (PlanningConfiguration) not concrete implementations
 */
MotionPlanningResult executeMotionPlanning(const PlanningConfiguration& config);

/**
 * @brief YAML loader for planning configuration
 */
bool loadYamlConfiguration(const std::string& yaml_file, PlanningConfiguration& config);

} // namespace vamp_ompl

/**
 * @brief Implementation of core functions (Single Responsibility Principle)
 */
namespace vamp_ompl {

/**
 * @brief Environment factory implementation (configurable obstacles)
 */
inline std::unique_ptr<EnvironmentFactory> createEnvironmentFactory(const std::vector<ObstacleConfig>& obstacles)
{
    auto factory = std::make_unique<ConfigurableEnvironmentFactory>();
    factory->setObstacles(obstacles);
    
    if (obstacles.empty()) {
        factory->setMetadata("Empty Environment", "Environment with no obstacles");
        std::cout << "🏭 Using empty environment (0 obstacles)" << std::endl;
    } else {
        factory->setMetadata("Custom Environment", "Environment with obstacle configuration");
        std::cout << "🔧 Using obstacle configuration (" 
                  << obstacles.size() << " obstacles)" << std::endl;
    }
    
    return factory;
}

/**
 * @brief Robot type dispatcher (Template Pattern for compile time dispatch)
 */
template<typename Func>
auto dispatchByRobotType(const std::string& robot_name, Func&& func) -> decltype(func.template operator()<vamp::robots::Panda>())
{
    if (robot_name == "panda") {
        return func.template operator()<vamp::robots::Panda>();
    } else if (robot_name == "ur5") {
        return func.template operator()<vamp::robots::UR5>();
    } else if (robot_name == "fetch") {
        return func.template operator()<vamp::robots::Fetch>();
    } else {
        const auto supportedRobots = constants::getSupportedRobots();
        std::string robotList;
        for (size_t i = 0; i < supportedRobots.size(); ++i) {
            robotList += supportedRobots[i];
            if (i < supportedRobots.size() - 1) robotList += ", ";
        }
        throw VampConfigurationError("Unsupported robot: " + robot_name + 
                                   ". Available: " + robotList);
    }
}

/**
 * @brief Core robot motion planning execution
 */
template<typename Robot>
MotionPlanningResult planRobotMotion(const PlanningConfiguration& config)
{
    MotionPlanningResult result;
    
    try {
        // Validate configuration
        if (!config.isValid()) {
            throw VampConfigurationError("Invalid configuration: " + config.getValidationErrors());
        }
        
        // Create robot configuration and environment
        auto robot_config = createRobotConfiguration<Robot>(config.robot_name, 
                                                            config.start_config, 
                                                            config.goal_config);
        auto env_factory = createEnvironmentFactory(config.obstacles);
        
        // Create planner
        auto planner = createVampOMPLPlanner(std::move(robot_config), std::move(env_factory));
        
        // Initialize planner
        planner->initialize();
        
        // Configure path writing
        PlanningConfig planning_config = config.planning;
        planning_config.write_path = config.save_path;
        
        // Execute planning
        result.planning_result = planner->plan(planning_config);
        
        // Set robot name for visualization
        result.robot_name = config.robot_name;
        
        // Set solution file path if path was written
        if (config.save_path && result.success()) {
            result.solution_file_path = result.planning_result.solution_file_path;
        }
        
    } catch (const std::exception& e) {
        result.planning_result.success = false;
        result.planning_result.error_message = e.what();
    }
    
    return result;
}

/**
 * @brief Main motion planning function implementation (Dependency Inversion Principle)
 */
inline MotionPlanningResult executeMotionPlanning(const PlanningConfiguration& config)
{
    return dispatchByRobotType(config.robot_name, [&config]<typename Robot>() {
        return planRobotMotion<Robot>(config);
    });
}


/**
 * @brief YAML configuration loader
 */
inline bool loadYamlConfiguration(const std::string& yaml_file, PlanningConfiguration& config)
{
    return VampUtils::loadYamlConfig(yaml_file, config);
}

} // namespace vamp_ompl