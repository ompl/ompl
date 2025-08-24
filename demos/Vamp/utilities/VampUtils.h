/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sahruday Patti */

#pragma once

#include "VampOMPLInterfaces.h"
#include <vamp/collision/factory.hh>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <cstdlib>
#include <stdexcept>

namespace vamp_ompl {

// ==== CONSTANTS ====
namespace constants {
    constexpr size_t DEFAULT_PATH_WAYPOINTS = 100;
    constexpr double DEFAULT_PLANNING_TIME = 2.0;
    constexpr double DEFAULT_SIMPLIFICATION_TIME = 1.0;
    constexpr float DEFAULT_SPHERE_RADIUS = 0.15f;
    constexpr float DEFAULT_CAPSULE_LENGTH = 0.2f;
    constexpr std::array<float, 3> DEFAULT_HALF_EXTENTS = {0.1f, 0.1f, 0.1f};
    constexpr float DEFAULT_POINT_RADIUS = 0.0025f;
    
    // RRT-Connect specific defaults
    constexpr double DEFAULT_RRT_RANGE = 0.3;
    constexpr bool DEFAULT_INTERMEDIATE_STATES = false;
}

// ==== ERROR HANDLING ====
class VampConfigurationError : public std::runtime_error {
public:
    explicit VampConfigurationError(const std::string& message) 
        : std::runtime_error("Configuration Error: " + message) {}
};

class VampYamlError : public VampConfigurationError {
public:
    explicit VampYamlError(const std::string& message) 
        : VampConfigurationError("YAML parsing failed: " + message) {}
};

// ==== POINTCLOUD UTILITIES ====
class PointcloudLoader {
public:
    using Point = std::array<float, 3>;
    
    static std::vector<Point> loadPointcloud(const std::string& filename) {
        std::filesystem::path filepath(filename);
        std::string extension = filepath.extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        
        if (extension == ".xyz") {
            return loadXYZ(filename);
        } else if (extension == ".ply") {
            return loadPLY(filename);
        } else if (extension == ".pcd") {
            return loadPCD(filename);
        }
        
        throw VampConfigurationError("Unsupported pointcloud format: " + extension);
    }

private:

    static std::vector<Point> loadXYZ(const std::string& filename) {
        std::vector<Point> points;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            throw VampConfigurationError("Failed to open XYZ file: " + filename);
        }
        
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Point point;
            if (iss >> point[0] >> point[1] >> point[2]) {
                points.push_back(point);
            }
        }
        
        return points;
    }

    static std::vector<Point> loadPLY(const std::string& filename) {
        std::vector<Point> points;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            throw VampConfigurationError("Failed to open PLY file: " + filename);
        }
        
        std::string line;
        size_t vertex_count = 0;
        bool in_header = true;
        
        while (std::getline(file, line)) {
            if (in_header) {
                if (line.find("element vertex") != std::string::npos) {
                    std::istringstream iss(line);
                    std::string element, vertex;
                    iss >> element >> vertex >> vertex_count;
                }
                if (line == "end_header") {
                    in_header = false;
                }
            } else {
                std::istringstream iss(line);
                Point point;
                if (iss >> point[0] >> point[1] >> point[2]) {
                    points.push_back(point);
                    if (points.size() >= vertex_count) break;
                }
            }
        }
        
        return points;
    }

    static std::vector<Point> loadPCD(const std::string& filename) {
        std::vector<Point> points;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            throw VampConfigurationError("Failed to open PCD file: " + filename);
        }
        
        std::string line;
        size_t point_count = 0;
        bool in_header = true;
        
        while (std::getline(file, line)) {
            if (in_header) {
                if (line.find("POINTS") != std::string::npos) {
                    std::istringstream iss(line);
                    std::string points_keyword;
                    iss >> points_keyword >> point_count;
                }
                if (line.find("DATA") != std::string::npos) {
                    in_header = false;
                }
            } else {
                std::istringstream iss(line);
                Point point;
                if (iss >> point[0] >> point[1] >> point[2]) {
                    points.push_back(point);
                    if (points.size() >= point_count) break;
                }
            }
        }
        
        return points;
    }
};

// ==== FILE UTILITIES ====
namespace FileUtils {
    static std::string findFile(const std::string& target_filename, const std::vector<std::string>& search_paths) {
        for (const auto& search_path : search_paths) {
            std::filesystem::path candidate_path = std::filesystem::path(search_path) / target_filename;
            if (std::filesystem::exists(candidate_path)) {
                return candidate_path.string();
            }
        }
        return "";
    }

    static std::string findYamlFile(const std::string& yaml_filename) {
        std::vector<std::string> search_paths = {
            ".",
            "./config",
            "../config",
            "./demos/Vamp/config",
            "../demos/Vamp/config"
        };
        return findFile(yaml_filename, search_paths);
    }

    static std::string findPointcloudFile(const std::string& pointcloud_filename) {
        std::vector<std::string> search_paths = {
            ".",
            "./pointclouds",
            "../pointclouds",
            "./demos/Vamp/pointclouds",
            "../demos/Vamp/pointclouds"
        };
        return findFile(pointcloud_filename, search_paths);
    }
}

// ==== CONFIGURABLE ENVIRONMENT FACTORY ====
class ConfigurableEnvironmentFactory : public EnvironmentFactory {
private:
    std::vector<ObstacleConfig> obstacle_configurations_;
    std::string environment_name_;
    std::string environment_description_;
    std::string robot_name_;
    
public:
    ConfigurableEnvironmentFactory(const std::vector<ObstacleConfig>& obstacle_configurations = {},
                                 const std::string& environment_name = "Environment",
                                 const std::string& environment_description = "Explicit obstacle environment",
                                 const std::string& robot_name = "")
        : obstacle_configurations_(obstacle_configurations), environment_name_(environment_name), 
          environment_description_(environment_description), robot_name_(robot_name) {}
    
    void setObstacles(const std::vector<ObstacleConfig>& obstacle_configurations) {
        obstacle_configurations_ = obstacle_configurations;
    }
    
    void addObstacle(const ObstacleConfig& obstacle_configuration) {
        obstacle_configurations_.push_back(obstacle_configuration);
    }
    
    void setMetadata(const std::string& environment_name, const std::string& environment_description) {
        environment_name_ = environment_name;
        environment_description_ = environment_description;
    }
    
    void setRobotName(const std::string& robot_name) {
        robot_name_ = robot_name;
    }
    
    auto create_environment() -> vamp::collision::Environment<float> override {
        vamp::collision::Environment<float> vamp_environment;
        
        for (const auto& obstacle_config : obstacle_configurations_) {
            if (obstacle_config.type == "sphere") {
                vamp_environment.spheres.emplace_back(
                    vamp::collision::factory::sphere::array(obstacle_config.position, obstacle_config.radius)
                );
                
            } else if (obstacle_config.type == "cuboid") {
                vamp_environment.cuboids.emplace_back(
                    vamp::collision::factory::cuboid::array(
                        obstacle_config.position, 
                        obstacle_config.orientation_euler_xyz, 
                        obstacle_config.half_extents
                    )
                );
                
            } else if (obstacle_config.type == "capsule") {
                vamp_environment.capsules.emplace_back(
                    vamp::collision::factory::capsule::center::array(
                        obstacle_config.position, 
                        obstacle_config.orientation_euler_xyz, 
                        obstacle_config.radius, 
                        obstacle_config.length
                    )
                );
                
            } else if (obstacle_config.type == "pointcloud") {
                try {
                    auto points = PointcloudLoader::loadPointcloud(obstacle_config.pointcloud_file);
                    if (!points.empty()) {
                        // Use default radii for CAPT construction
                        float r_min = 0.01f;  // Default minimum radius
                        float r_max = 1.0f;   // Default maximum radius
                        vamp_environment.pointclouds.emplace_back(points, r_min, r_max, obstacle_config.point_radius);
                        std::cout << "Loaded pointcloud: " << points.size() << " points" << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error loading pointcloud " << obstacle_config.pointcloud_file 
                              << ": " << e.what() << std::endl;
                }
            }
        }
        
        vamp_environment.sort();
        return vamp_environment;
    }
    
    auto get_environment_name() const -> std::string override {
        return environment_name_;
    }
    
    auto get_description() const -> std::string override {
        return environment_description_;
    }
};

// ==== YAML UTILITIES ====
namespace YamlUtils {
    template<typename TargetType>
    static TargetType safeConvertYamlValue(const YAML::Node& yaml_node, const TargetType& default_value = TargetType{}) {
        try {
            return yaml_node.as<TargetType>();
        } catch (const YAML::Exception& e) {
            return default_value;
        }
    }

    static void parseRobotSection(const YAML::Node& robot_node, PlanningConfiguration& planning_configuration) {
        if (robot_node["name"]) {
            planning_configuration.robot_name = robot_node["name"].as<std::string>();
        }
        if (robot_node["description"]) {
            planning_configuration.description = robot_node["description"].as<std::string>();
        }
    }

    static void parsePlannerSection(const YAML::Node& planner_node, PlanningConfiguration& planning_configuration) {
        if (planner_node["name"]) {
            planning_configuration.planning.planner_name = planner_node["name"].as<std::string>();
        }
        if (planner_node["planning_time"]) {
            planning_configuration.planning.planning_time = planner_node["planning_time"].as<double>();
        }
        if (planner_node["simplification_time"]) {
            planning_configuration.planning.simplification_time = planner_node["simplification_time"].as<double>();
        }
        if (planner_node["optimize_path"]) {
            planning_configuration.planning.optimize_path = planner_node["optimize_path"].as<bool>();
        }
        if (planner_node["parameters"]) {
            for (const auto& param : planner_node["parameters"]) {
                std::string key = param.first.as<std::string>();
                std::string value = param.second.as<std::string>();
                planning_configuration.planning.planner_parameters[key] = value;
            }
        }
    }

    static ObstacleConfig parseObstacle(const YAML::Node& obstacle_node) {
        ObstacleConfig obstacle;
        
        obstacle.type = obstacle_node["type"].as<std::string>();
        
        if (obstacle_node["name"]) {
            obstacle.name = obstacle_node["name"].as<std::string>();
        }
        
        if (obstacle.type == "sphere") {
            auto position = obstacle_node["position"].as<std::vector<float>>();
            obstacle.position = {position[0], position[1], position[2]};
            obstacle.radius = obstacle_node["radius"].as<float>();
        } else if (obstacle.type == "cuboid") {
            auto position = obstacle_node["position"].as<std::vector<float>>();
            obstacle.position = {position[0], position[1], position[2]};
            auto half_extents = obstacle_node["half_extents"].as<std::vector<float>>();
            obstacle.half_extents = {half_extents[0], half_extents[1], half_extents[2]};
            if (obstacle_node["orientation_euler_xyz"]) {
                auto orientation = obstacle_node["orientation_euler_xyz"].as<std::vector<float>>();
                obstacle.orientation_euler_xyz = {orientation[0], orientation[1], orientation[2]};
            }
        } else if (obstacle.type == "capsule") {
            auto position = obstacle_node["position"].as<std::vector<float>>();
            obstacle.position = {position[0], position[1], position[2]};
            obstacle.radius = obstacle_node["radius"].as<float>();
            obstacle.length = obstacle_node["length"].as<float>();
            if (obstacle_node["orientation_euler_xyz"]) {
                auto orientation = obstacle_node["orientation_euler_xyz"].as<std::vector<float>>();
                obstacle.orientation_euler_xyz = {orientation[0], orientation[1], orientation[2]};
            }
        } else if (obstacle.type == "pointcloud") {
            obstacle.pointcloud_file = obstacle_node["pointcloud_file"].as<std::string>();
            obstacle.point_radius = safeConvertYamlValue<float>(obstacle_node["point_radius"], constants::DEFAULT_POINT_RADIUS);
        }
        
        return obstacle;
    }

    static void parseObstaclesSection(const YAML::Node& obstacles_node, PlanningConfiguration& planning_configuration) {
        if (!obstacles_node.IsSequence()) {
            throw VampYamlError("obstacles section must be a sequence");
        }
        
        for (const auto& obstacle_node : obstacles_node) {
            auto obstacle = parseObstacle(obstacle_node);
            planning_configuration.obstacles.push_back(obstacle);
        }
    }

    static std::vector<float> parseConfigurationArray(const YAML::Node& configuration_array_node) {
        if (!configuration_array_node.IsSequence()) {
            throw VampYamlError("Configuration must be an array");
        }
        
        std::vector<float> configuration;
        for (const auto& value : configuration_array_node) {
            configuration.push_back(value.as<float>());
        }
        
        return configuration;
    }
}

/**
 * @brief Main VampUtils class providing utilities for VAMP-OMPL integration
 * 
 * This class serves as the main interface for all VAMP utility functions, providing
 * a consistent API for file operations, configuration validation, output
 * formatting, and environment creation.
 */
class VampUtils {
public:
    // ========== ENVIRONMENT FACTORY ==========
    
    /**
     * @brief Create environment factory with obstacles
     * @param obstacles Vector of obstacle configurations
     * @param robot_name Robot name for metadata
     * @return Unique pointer to configured environment factory
     */
    static std::unique_ptr<EnvironmentFactory> createEnvironmentFactory(
        const std::vector<ObstacleConfig>& obstacles,
        const std::string& robot_name = "") {
        
        auto factory = std::make_unique<ConfigurableEnvironmentFactory>();
        factory->setObstacles(obstacles);
        factory->setRobotName(robot_name);
        
        if (obstacles.empty()) {
            factory->setMetadata("Empty Environment", "Environment with no obstacles");
            std::cout << " Using empty environment (0 obstacles)" << std::endl;
        } else {
            factory->setMetadata("Custom Environment", "Environment with obstacle configuration");
            std::cout << " Using obstacle configuration (" 
                      << obstacles.size() << " obstacles)" << std::endl;
        }
        
        return factory;
    }

    // ========== VALIDATION UTILITIES ==========
    
    /**
     * @brief Validate configuration dimensions against robot requirements
     */
    static void validateConfigurationDimension(
        const std::vector<float>& config,
        std::size_t expected_dimension,
        const std::string& config_type,
        const std::string& robot_name) {
        
        if (config.size() != expected_dimension) {
            throw VampConfigurationError(config_type + " configuration has " + 
                std::to_string(config.size()) + " dimensions, expected " + 
                std::to_string(expected_dimension) + " for robot " + robot_name);
        }
    }
    
    /**
     * @brief Validate configuration limits
     */
    static void validateConfigurationLimits(
        const std::vector<float>& config,
        const std::vector<std::pair<float, float>>& limits,
        const std::string& config_type,
        const std::string& robot_name) {
        
        for (size_t i = 0; i < config.size(); ++i) {
            if (config[i] < limits[i].first || config[i] > limits[i].second) {
                throw VampConfigurationError(config_type + " joint " + std::to_string(i) + 
                    " (" + std::to_string(config[i]) + ") outside limits [" + 
                    std::to_string(limits[i].first) + "," + std::to_string(limits[i].second) + 
                    "] for robot " + robot_name);
            }
        }
    }

    // ========== OUTPUT AND PRINTING ==========
    
    static void printUsage(const char* program_name) {
        std::cout << "Usage: " << program_name << " [config.yaml]" << std::endl;
        std::cout << "  config.yaml: Optional YAML configuration file" << std::endl;
        std::cout << "  If no config provided, uses programmatic configuration" << std::endl;
    }
    
    static void printConfigSummary(const PlanningConfiguration& planning_configuration) {
        std::cout << "\n=== Configuration Summary ===" << std::endl;
        std::cout << "Robot: " << planning_configuration.robot_name << std::endl;
        std::cout << "Planner: " << planning_configuration.planning.planner_name << std::endl;
        std::cout << "Planning time: " << planning_configuration.planning.planning_time << "s" << std::endl;
        std::cout << "Obstacles: " << planning_configuration.obstacles.size() << std::endl;
    }
    
    static void printResults(const MotionPlanningResult& motion_planning_result) {
        std::cout << "\n=== Planning Results ===" << std::endl;
        if (motion_planning_result.success()) {
            std::cout << "Planning: SUCCESS" << std::endl;
            std::cout << "Planning time: " << (motion_planning_result.planning_result.planning_time_us / 1000.0) << " ms" << std::endl;
            std::cout << "Path length: " << motion_planning_result.planning_result.path_length << " waypoints" << std::endl;
        } else {
            std::cout << "Planning: FAILED" << std::endl;
            if (!motion_planning_result.planning_result.error_message.empty()) {
                std::cout << "Error: " << motion_planning_result.planning_result.error_message << std::endl;
            }
        }
    }
    
    template<typename Robot>
    static void printPlannerConfiguration(const RobotConfig<Robot>& robot_configuration, 
                                        const EnvironmentFactory& environment_factory,
                                        bool is_initialized) {
        std::cout << "\n=== Planner Configuration ===" << std::endl;
        std::cout << "Robot: " << robot_configuration.get_robot_name() << std::endl;
        std::cout << "Environment: " << environment_factory.get_environment_name() << std::endl;
        std::cout << "Description: " << environment_factory.get_description() << std::endl;
        std::cout << "Initialized: " << (is_initialized ? "Yes" : "No") << std::endl;
        
        auto robot_joint_limits = robot_configuration.get_joint_limits();
        std::cout << "Joint limits:" << std::endl;
        for (size_t i = 0; i < robot_joint_limits.size(); ++i) {
            std::cout << "  Joint " << i << ": [" 
                      << robot_joint_limits[i].first << ", " 
                      << robot_joint_limits[i].second << "]" << std::endl;
        }
    }

    // ========== VISUALIZATION ==========
    
    static bool runVisualization(const MotionPlanningResult& motion_planning_result, const std::string& /* configuration_source */, const std::string& yaml_file = "") {
        if (!motion_planning_result.success()) {
            std::cout << "Cannot visualize: planning failed" << std::endl;
            return false;
        }
        
        if (motion_planning_result.solution_file_path.empty()) {
            std::cout << "Cannot visualize: no solution file written" << std::endl;
            return false;
        }
        
        // Find the visualization script in the demos/Vamp/visualization directory
        std::string script_path = "../demos/Vamp/visualization/visualize_solution.py";
        
        // Check if script exists, if not try alternative paths
        std::ifstream script_file(script_path);
        if (!script_file.good()) {
            // Try relative path from build directory
            script_path = "demos/Vamp/visualization/visualize_solution.py";
            script_file.open(script_path);
            if (!script_file.good()) {
                // Try absolute path construction
                script_path = "python3 -m demos.Vamp.visualization.visualize_solution";
            }
        }
        script_file.close();
        
        std::string command = "python3 " + script_path + " " + motion_planning_result.solution_file_path;
        if (!yaml_file.empty()) {
            // Resolve YAML file path to ensure it can be found by the visualization script
            std::string resolved_yaml_path = findYamlFile(yaml_file);
            if (!resolved_yaml_path.empty()) {
                command += " --yaml-config " + resolved_yaml_path;
            } else {
                // Fallback: try relative path from demos/Vamp
                command += " --yaml-config " + yaml_file;
            }
        }
        std::cout << "Launching visualization: " << command << std::endl;
        
        int result = std::system(command.c_str());
        return result == 0;
    }

    // ========== FILE OPERATIONS ==========
    
    static std::string findYamlFile(const std::string& yaml_filename) {
        return FileUtils::findYamlFile(yaml_filename);
    }
    
    static std::string findFile(const std::string& target_filename, const std::vector<std::string>& search_paths) {
        return FileUtils::findFile(target_filename, search_paths);
    }

    // ========== YAML CONFIGURATION ==========
    
    static bool loadYamlConfig(const std::string& yaml_filename, PlanningConfiguration& planning_configuration) {
        try {
            std::string resolved_path = findYamlFile(yaml_filename);
            if (resolved_path.empty()) {
                std::cerr << "YAML file not found: " << yaml_filename << std::endl;
                return false;
            }
            
            YAML::Node yaml_configuration = YAML::LoadFile(resolved_path);
            
            // Parse robot section
            if (yaml_configuration["robot"]) {
                YamlUtils::parseRobotSection(yaml_configuration["robot"], planning_configuration);
            }
            
            // Parse planner section
            if (yaml_configuration["planner"]) {
                YamlUtils::parsePlannerSection(yaml_configuration["planner"], planning_configuration);
            }
            
            // Parse start and goal configurations
            if (yaml_configuration["start_config"]) {
                planning_configuration.start_config = YamlUtils::parseConfigurationArray(yaml_configuration["start_config"]);
            }
            if (yaml_configuration["goal_config"]) {
                planning_configuration.goal_config = YamlUtils::parseConfigurationArray(yaml_configuration["goal_config"]);
            }
            
            // Parse obstacles
            if (yaml_configuration["obstacles"]) {
                YamlUtils::parseObstaclesSection(yaml_configuration["obstacles"], planning_configuration);
            }
            
            // Parse output settings
            if (yaml_configuration["output"]) {
                if (yaml_configuration["output"]["write_path"]) {
                    planning_configuration.save_path = yaml_configuration["output"]["write_path"].as<bool>();
                }
                if (yaml_configuration["output"]["description"]) {
                    planning_configuration.description = yaml_configuration["output"]["description"].as<std::string>();
                }
            }
            
            // Parse visualization settings
            if (yaml_configuration["visualization"]) {
                auto& viz = planning_configuration.visualization;
                if (yaml_configuration["visualization"]["urdf_path"]) {
                    viz.urdf_path = yaml_configuration["visualization"]["urdf_path"].as<std::string>();
                }
                if (yaml_configuration["visualization"]["expected_joints"]) {
                    viz.expected_joints = yaml_configuration["visualization"]["expected_joints"].as<int>();
                }
                if (yaml_configuration["visualization"]["base_position"]) {
                    auto pos = yaml_configuration["visualization"]["base_position"].as<std::vector<float>>();
                    viz.base_position = {pos[0], pos[1], pos[2]};
                }
                if (yaml_configuration["visualization"]["base_orientation"]) {
                    auto orient = yaml_configuration["visualization"]["base_orientation"].as<std::vector<float>>();
                    viz.base_orientation = {orient[0], orient[1], orient[2]};
                }
                if (yaml_configuration["visualization"]["use_fixed_base"]) {
                    viz.use_fixed_base = yaml_configuration["visualization"]["use_fixed_base"].as<bool>();
                }
                if (yaml_configuration["visualization"]["description"]) {
                    viz.description = yaml_configuration["visualization"]["description"].as<std::string>();
                }
            }
            
            return true;
            
        } catch (const YAML::Exception& e) {
            throw VampYamlError("Failed to parse YAML file '" + yaml_filename + "': " + e.what());
        } catch (const std::exception& e) {
            throw VampConfigurationError("Failed to load configuration from '" + yaml_filename + "': " + e.what());
        }
    }
};

} // namespace vamp_ompl