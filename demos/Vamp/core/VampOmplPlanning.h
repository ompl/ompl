/**
 * @file VampOmplPlanning.h
 * @brief Comprehensive VAMP-OMPL integration header
 * 
 * This header provides complete access to VAMP-OMPL functionality including:
 * - High-level planning interface (executeMotionPlanning)
 * - Robot registry system with built-in and custom robot support
 * - Integrated planner management (RRT-Connect, BIT*, PRM, custom planners)
 * - SIMD-accelerated collision detection and motion validation
 * - Configuration structures and utilities
 * - Benchmarking infrastructure
 * - Visualization support
 * 
 * Usage:
 * ```cpp
 * #include "VampOmplPlanning.h"
 * 
 * // All VAMP-OMPL functionality is now available
 * vamp_ompl::PlanningConfiguration config;
 * auto result = vamp_ompl::executeMotionPlanning(config);
 * ```
 */
#pragma once

// ========== CORE VAMP-OMPL INTERFACES ==========
#include "VampOMPLInterfaces.h"      // Core interfaces and data structures
#include "VampValidators.h"          // SIMD-accelerated collision detection
#include "OMPLPlanningContext.h"     // Unified OMPL integration with planner management
#include "VampOMPLPlanner.h"         // Main planning facade
#include "VampUtils.h"               // Utilities, YAML loading, environment creation
#include "VampRobotRegistry.h"       // Type-safe robot registry system

// ========== BENCHMARKING SUPPORT ==========
#include "VampBenchmarkManager.h"    // OMPL-compliant benchmarking infrastructure

// ========== STANDARD LIBRARY INCLUDES ==========
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <chrono>

// ========== VAMP CORE INCLUDES ==========
#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/vector.hh>

// ========== OMPL INCLUDES ==========
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/tools/benchmark/Benchmark.h>

// ========== BUILT-IN ROBOT SUPPORT ==========
#include <vamp/robots/panda.hh>      // Franka Emika Panda 7-DOF
#include <vamp/robots/ur5.hh>        // Universal Robots UR5 6-DOF
#include <vamp/robots/fetch.hh>      // Fetch Robotics 8-DOF mobile manipulator

// ========== YAML CONFIGURATION SUPPORT ==========
#include <yaml-cpp/yaml.h>

namespace vamp_ompl {

// ========== HIGH-LEVEL PLANNING INTERFACE ==========

/**
 * @brief Execute motion planning with comprehensive configuration
 * 
 * This is the main entry point for VAMP-OMPL motion planning. It provides
 * a unified interface that handles robot selection, environment creation,
 * planner configuration, and execution through the registry system.
 * 
 * @param config Complete planning configuration including robot, environment, and planner settings
 * @return Motion planning result with success status, timing, and solution path
 * 
 * Example:
 * ```cpp
 * PlanningConfiguration config;
 * config.robot_name = "panda";
 * config.start_config = {0, 0, 0, 0, 0, 0, 0};
 * config.goal_config = {1, 1, 1, 1, 1, 1, 1};
 * config.planning.planner_name = "RRT-Connect";
 * config.planning.planning_time = 5.0;
 * 
 * auto result = executeMotionPlanning(config);
 * if (result.success()) {
 *     std::cout << "Planning succeeded!" << std::endl;
 * }
 * ```
 */
MotionPlanningResult executeMotionPlanning(const PlanningConfiguration& config);

/**
 * @brief Write visualization configuration to solution file
 * 
 * Embeds visualization metadata into the solution path file for use
 * by the visualization system.
 * 
 * @param solution_file_path Path to the solution file
 * @param robot_name Name of the robot for visualization
 * @param viz_config Visualization configuration parameters
 */
void writeVisualizationConfig(const std::string& solution_file_path, 
                            const std::string& robot_name,
                            const PlanningConfiguration::VisualizationConfig& viz_config);

// ========== CONVENIENCE FACTORY FUNCTIONS ==========

/**
 * @brief Create a VAMP-OMPL planner for specific robot type
 * 
 * Factory function that creates a fully configured planner instance
 * for direct use without the registry system.
 * 
 * @tparam Robot VAMP robot type (e.g., vamp::robots::Panda)
 * @param robot_configuration Robot configuration with limits and poses
 * @param environment_factory Factory for creating collision environments
 * @return Unique pointer to configured planner
 */
template<typename Robot>
auto createVampOmplPlanner(std::unique_ptr<RobotConfig<Robot>> robot_configuration,
                          std::unique_ptr<EnvironmentFactory> environment_factory) 
    -> std::unique_ptr<VampOMPLPlanner<Robot>> {
    
    return std::make_unique<VampOMPLPlanner<Robot>>(
        std::move(robot_configuration), std::move(environment_factory));
}

/**
 * @brief Create robot configuration for specific robot type
 * 
 * Convenience function for creating typed robot configurations.
 * 
 * @tparam Robot VAMP robot type
 * @param robot_name robot name
 * @param start_config Start joint configuration
 * @param goal_config Goal joint configuration
 * @return Unique pointer to robot configuration
 */
template<typename Robot>
auto createRobotConfiguration(const std::string& robot_name,
                             const std::vector<float>& start_config,
                             const std::vector<float>& goal_config)
    -> std::unique_ptr<RobotConfiguration<Robot>> {
    
    return std::make_unique<RobotConfiguration<Robot>>(
        robot_name, start_config, goal_config);
}

// ========== PLANNER REGISTRATION CONVENIENCE ==========

/**
 * @brief Register a custom planner for all robot types
 * 
 * Convenience function that registers a planner for use with any robot type.
 * The planner will be available through the string-based planner selection.
 * 
 * @param name Unique planner identifier
 * @param allocator Factory function that creates the planner
 * 
 * Example:
 * ```cpp
 * registerCustomPlanner("MyRRT*", 
 *     [](const ob::SpaceInformationPtr& si, const auto& params) {
 *         auto planner = std::make_shared<og::RRTstar>(si);
 *         // Configure parameters...
 *         return planner;
 *     });
 * ```
 */
template<typename Robot>
void registerCustomPlanner(const std::string& name, 
                          std::function<ob::PlannerPtr(const ob::SpaceInformationPtr&, 
                                                      const std::map<std::string, std::string>&)> allocator) {
    OMPLPlanningContext<Robot>::registerPlanner(name, std::move(allocator));
}

// ========== BUILT-IN ROBOT ACCESS ==========

/**
 * @brief Get list of all built-in robots
 * 
 * @return Vector of robot names that are automatically registered
 */
inline std::vector<std::string> getBuiltInRobots() {
    return {"panda", "ur5", "fetch"};
}

/**
 * @brief Get list of all built-in planners
 * 
 * @return Vector of planner names that are automatically registered
 */
inline std::vector<std::string> getBuiltInPlanners() {
    return {"RRT-Connect", "BIT*", "PRM"};
}

/**
 * @brief Check if a robot is available
 * 
 * @param robot_name Robot identifier to check
 * @return true if robot is registered and available
 */
inline bool isRobotAvailable(const std::string& robot_name) {
    return RobotRegistry::getInstance().isRobotRegistered(robot_name);
}

/**
 * @brief Get robot metadata
 * 
 * @param robot_name Robot identifier
 * @return Robot metadata including dimension, description, etc.
 * @throws VampConfigurationError if robot not found
 */
inline RobotMetadata getRobotInfo(const std::string& robot_name) {
    return RobotRegistry::getInstance().getRobotMetadata(robot_name);
}

// ========== CONFIGURATION HELPERS ==========

/**
 * @brief Create basic planning configuration
 * 
 * Creates a minimal configuration with sensible defaults that can be
 * customized as needed.
 * 
 * @param robot_name Robot to use for planning
 * @param planner_name Planner to use (default: "RRT-Connect")
 * @param planning_time Time limit in seconds (default: 5.0)
 * @return Basic planning configuration
 */
inline PlanningConfiguration createBasicConfiguration(
    const std::string& robot_name,
    const std::string& planner_name = "RRT-Connect",
    double planning_time = 5.0) {
    
    PlanningConfiguration config;
    config.robot_name = robot_name;
    config.planning.planner_name = planner_name;
    config.planning.planning_time = planning_time;
    config.planning.simplification_time = 1.0;
    config.save_path = true;
    
    return config;
}

/**
 * @brief Load configuration from YAML file
 * 
 * Convenience wrapper around VampUtils::loadYamlConfig.
 * 
 * @param yaml_file Path to YAML configuration file
 * @return Loaded planning configuration
 * @throws VampYamlError if loading fails
 */
inline PlanningConfiguration loadConfigurationFromYaml(const std::string& yaml_file) {
    PlanningConfiguration config;
    if (!VampUtils::loadYamlConfig(yaml_file, config)) {
        throw VampYamlError("Failed to load configuration from: " + yaml_file);
    }
    return config;
}

// ========== BENCHMARKING CONVENIENCE ==========

/**
 * @brief Create benchmark manager for specific robot
 * 
 * @tparam Robot VAMP robot type
 * @param robot_config Robot configuration
 * @param env_factory Environment factory
 * @return Shared pointer to benchmark manager
 */
template<typename Robot>
auto createVampBenchmarkManager(std::unique_ptr<RobotConfig<Robot>> robot_config,
                               std::unique_ptr<EnvironmentFactory> env_factory)
    -> std::shared_ptr<benchmarking::VampBenchmarkManager<Robot>> {
    
    return benchmarking::createBenchmarkManager<Robot>(
        std::move(robot_config), std::move(env_factory));
}

} // namespace vamp_ompl

namespace vamp_ompl {

/**
 * @brief Implementation of executeMotionPlanning
 */
inline MotionPlanningResult executeMotionPlanning(const PlanningConfiguration& config)
{
    MotionPlanningResult result;
    
    try {
        auto& registry = RobotRegistry::getInstance();
        
        // Validate configuration
        if (!config.isValid()) {
            throw VampConfigurationError("Invalid configuration: " + config.getValidationErrors());
        }
        
        // Create robot configuration using registry
        auto robot_config = registry.createRobotConfig(
            config.robot_name, config.start_config, config.goal_config);
        
        // Create environment factory using VampUtils
        auto env_factory = VampUtils::createEnvironmentFactory(config.obstacles, config.robot_name);
        
        // Create planner using registry
        auto planner = registry.createPlanner(
            config.robot_name, std::move(robot_config), std::move(env_factory));
        
        // Initialize planner using registry
        registry.initializePlanner(config.robot_name, planner);
        
        // Configure path writing
        PlanningConfig planning_config = config.planning;
        planning_config.write_path = config.save_path;
        
        // Execute planning using registry
        result.planning_result = registry.executePlanning(
            config.robot_name, planner, planning_config);
        
        // Write visualization configuration to path file if path was written
        if (config.save_path && result.planning_result.success && !result.planning_result.solution_file_path.empty()) {
            writeVisualizationConfig(result.planning_result.solution_file_path, config.robot_name, config.visualization);
        }
        
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
 * @brief Implementation of writeVisualizationConfig
 */
inline void writeVisualizationConfig(const std::string& solution_file_path, 
                                   const std::string& robot_name,
                                   const PlanningConfiguration::VisualizationConfig& viz_config) {
    if (viz_config.urdf_path.empty()) return;  // Nothing to write
    
    // Read existing file content
    std::ifstream infile(solution_file_path);
    if (!infile.is_open()) return;
    
    std::string content((std::istreambuf_iterator<char>(infile)), std::istreambuf_iterator<char>());
    infile.close();
    
    // Prepare visualization configuration header
    std::ostringstream viz_header;
    viz_header << "# VISUALIZATION CONFIG:\n";
    viz_header << "# robot_name: " << robot_name << "\n";
    viz_header << "# urdf_path: " << viz_config.urdf_path << "\n";
    if (viz_config.expected_joints > 0) {
        viz_header << "# expected_joints: " << viz_config.expected_joints << "\n";
    }
    viz_header << "# base_position: [" << viz_config.base_position[0] << ", " 
              << viz_config.base_position[1] << ", " << viz_config.base_position[2] << "]\n";
    viz_header << "# base_orientation: [" << viz_config.base_orientation[0] << ", " 
              << viz_config.base_orientation[1] << ", " << viz_config.base_orientation[2] << "]\n";
    viz_header << "# use_fixed_base: " << (viz_config.use_fixed_base ? "true" : "false") << "\n";
    
    // Find position after the first comment line to insert visualization config
    size_t first_newline = content.find('\n');
    if (first_newline != std::string::npos) {
        content.insert(first_newline + 1, viz_header.str());
        
        // Write back to file
        std::ofstream outfile(solution_file_path);
        if (outfile.is_open()) {
            outfile << content;
            std::cout << " Visualization config written to: " << solution_file_path << std::endl;
        }
    }
}

} // namespace vamp_ompl 