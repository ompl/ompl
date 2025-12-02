/**
 * @file VampOMPLDemo.h
 * @brief VAMP-OMPL integration interface for vectorized motion planning
 * 
 * This file provides a demonstration of VAMP-OMPL integration,
 * showcasing vectorized motion planning with SIMD acceleration through the
 * unified robot registry system.
 */
#pragma once

#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include "OMPLPlanningContext.h"
#include "VampOMPLPlanner.h"
#include "VampUtils.h"
#include "VampRobotRegistry.h"

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
 * @brief  motion planning execution function
 */
MotionPlanningResult executeMotionPlanning(const PlanningConfiguration& config);

/**
 * @brief Write visualization configuration to solution file
 */
void writeVisualizationConfig(const std::string& solution_file_path, 
                            const std::string& robot_name,
                            const PlanningConfiguration::VisualizationConfig& viz_config);

} // namespace vamp_ompl

/**
 * @brief Implementation of core functions
 */
namespace vamp_ompl {



/**
 * @brief  motion planning execution (uses registry for all robots)
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
 * @brief Write visualization configuration to solution file
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