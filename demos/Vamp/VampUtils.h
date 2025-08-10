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
    constexpr double DEFAULT_SIMPLIFICATION_TIME = 1.0;  // Updated to match demo usage
    constexpr float DEFAULT_SPHERE_RADIUS = 0.15f;       // Updated to match demo usage
    constexpr float DEFAULT_CAPSULE_LENGTH = 0.2f;
    constexpr std::array<float, 3> DEFAULT_HALF_EXTENTS = {0.1f, 0.1f, 0.1f};
    constexpr float DEFAULT_POINT_RADIUS = 0.0025f;  // VAMP default point radius
    
    // RRT-Connect specific defaults
    constexpr double DEFAULT_RRT_RANGE = 0.3;
    constexpr bool DEFAULT_INTERMEDIATE_STATES = false;
    
    // Use RobotRegistry::getInstance().getRegisteredRobots() or dynamic robot support
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

/**
 * @brief Pointcloud loader supporting common formats (.xyz, .ply, .pcd)
 * 
 * This utility class provides cross-platform pointcloud loading with minimal dependencies.
 * Supports the most common formats used in robotics and 3D applications.
 */
class PointcloudLoader {
public:
    using Point = std::array<float, 3>;
    
    /**
     * @brief Load pointcloud from file (auto-detects format)
     * @param filename Path to pointcloud file
     * @return Vector of 3D points
     */
    static std::vector<Point> loadPointcloud(const std::string& filename) {
        std::string extension = std::filesystem::path(filename).extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        
        if (extension == ".xyz") {
            return loadXYZ(filename);
        } else if (extension == ".ply") {
            return loadPLY(filename);
        } else if (extension == ".pcd") {
            return loadPCD(filename);
        } else {
            throw VampConfigurationError("Unsupported pointcloud format: " + extension + 
                ". Supported: .xyz, .ply, .pcd");
        }
    }

private:
    /**
     * @brief Load simple XYZ format (x y z per line)
     */
    static std::vector<Point> loadXYZ(const std::string& filename) {
        std::vector<Point> points;
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw VampConfigurationError("Cannot open pointcloud file: " + filename);
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
    
    /**
     * @brief Load basic PLY format (ASCII only)
     */
    static std::vector<Point> loadPLY(const std::string& filename) {
        std::vector<Point> points;
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw VampConfigurationError("Cannot open PLY file: " + filename);
        }
        
        std::string line;
        bool in_header = true;
        size_t vertex_count = 0;
        
        while (std::getline(file, line) && in_header) {
            if (line.find("element vertex") == 0) {
                std::istringstream iss(line);
                std::string element, vertex;
                iss >> element >> vertex >> vertex_count;
            } else if (line == "end_header") {
                in_header = false;
            }
        }
        
        points.reserve(vertex_count);
        for (size_t i = 0; i < vertex_count && std::getline(file, line); ++i) {
            std::istringstream iss(line);
            Point point;
            if (iss >> point[0] >> point[1] >> point[2]) {
                points.push_back(point);
            }
        }
        return points;
    }
    
    /**
     * @brief Load basic PCD format (ASCII only)
     */
    static std::vector<Point> loadPCD(const std::string& filename) {
        std::vector<Point> points;
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw VampConfigurationError("Cannot open PCD file: " + filename);
        }
        
        std::string line;
        bool in_header = true;
        size_t point_count = 0;
        
        while (std::getline(file, line) && in_header) {
            if (line.find("POINTS") == 0) {
                std::istringstream iss(line);
                std::string points_keyword;
                iss >> points_keyword >> point_count;
            } else if (line.find("DATA") == 0) {
                in_header = false;
            }
        }
        
        points.reserve(point_count);
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Point point;
            if (iss >> point[0] >> point[1] >> point[2]) {
                points.push_back(point);
            }
        }
        return points;
    }
};

/**
 * @brief File utilities for locating configuration and script files
 * 
 * This utility class implements the Strategy pattern for file location,
 * providing flexible search mechanisms across different deployment scenarios.
 * 
 * - Encapsulates file system interaction complexity
 * - Provides consistent error handling across file operations
 * - Supports multiple deployment scenarios (development, build, install)
 * - Demonstrates robust error reporting with actionable messages
 */
class FileLocator {
public:
    /**
     * @brief Find file in common locations using search strategy pattern
     * 
     *  Note: This method demonstrates the Strategy pattern applied
     * to file location. Different search strategies can be applied based on
     * deployment context without changing client code.
     */
    static std::string findFile(const std::string& targetFilename, const std::vector<std::string>& searchPaths) {
        for (const auto& candidatePath : searchPaths) {
            std::ifstream testFileStream(candidatePath);
            if (testFileStream.is_open()) {
                return candidatePath;
            }
        }
        
        std::string errorMessage = "File not found: " + targetFilename + "\nSearched paths:\n";
        for (const auto& searchPath : searchPaths) errorMessage += "  - " + searchPath + "\n";
        throw VampConfigurationError(errorMessage);
    }
    
    /**
     * @brief Find YAML config file in standard locations
     */
    static std::string findYamlFile(const std::string& yamlFilename) {
        return findFile(yamlFilename, {
            yamlFilename,                                    // Direct path
            "config/" + yamlFilename,                        // Local config dir
            "demos/Vamp/config/" + yamlFilename,             // From project root
            "../demos/Vamp/config/" + yamlFilename,          // From build dir
            "../../demos/Vamp/config/" + yamlFilename        // From deeper build dirs
        });
    }
    
    /**
     * @brief Find pointcloud file in standard locations
     */
    static std::string findPointcloudFile(const std::string& pointcloudFilename) {
        return findFile(pointcloudFilename, {
            pointcloudFilename,                               // Direct path
            "demos/Vamp/" + pointcloudFilename,              // From project root
            "../demos/Vamp/" + pointcloudFilename,           // From build dir
            "../../demos/Vamp/" + pointcloudFilename,        // From deeper build dirs
            "data/" + pointcloudFilename,                    // Local data dir
            "config/" + pointcloudFilename                   // Local config dir
        });
    }
};

/**
 * @brief Single configurable environment factory
 * 
 * This factory creates VAMP environments from explicit obstacle configurations.
 * No hardcoded or named environments are supported - all obstacles must be explicitly defined.
 * 
 * Performance Features:
 * - Lazy evaluation: Environment creation deferred until needed
 * - Memory efficient: No unnecessary object duplication
 * - Type-safe obstacle creation using modern C++ patterns
 */
class ConfigurableEnvironmentFactory : public EnvironmentFactory {
private:
    std::vector<ObstacleConfig> m_obstacleConfigurations;
    std::string m_environmentName;
    std::string m_environmentDescription;
    std::string m_robotName; // Add robot name for CAPT radii
    
public:
    /**
     * @brief Constructor with explicit obstacles list
     */
    ConfigurableEnvironmentFactory(const std::vector<ObstacleConfig>& obstacleConfigurations = {},
                                 const std::string& environmentName = "Environment",
                                 const std::string& environmentDescription = "Explicit obstacle environment",
                                 const std::string& robotName = "")
        : m_obstacleConfigurations(obstacleConfigurations), m_environmentName(environmentName), 
          m_environmentDescription(environmentDescription), m_robotName(robotName)
    {
    }
    
    /**
     * @brief Set obstacles for this environment
     */
    void setObstacles(const std::vector<ObstacleConfig>& obstacleConfigurations) {
        m_obstacleConfigurations = obstacleConfigurations;
    }
    
    /**
     * @brief Add a single obstacle
     */
    void addObstacle(const ObstacleConfig& obstacleConfiguration) {
        m_obstacleConfigurations.push_back(obstacleConfiguration);
    }
    
    /**
     * @brief Clear all obstacles (creates empty environment)
     */
    void clearObstacles() {
        m_obstacleConfigurations.clear();
    }
    
    /**
     * @brief Set robot name for robot-specific CAPT radii
     */
    void setRobotName(const std::string& robotName) {
        m_robotName = robotName;
    }
    
    /**
     * @brief Set environment metadata
     */
    void setMetadata(const std::string& name, const std::string& description) {
        m_environmentName = name;
        m_environmentDescription = description;
    }
    
    /**
     * @brief Create VAMP environment from explicit obstacle configurations
     * 
     * Note: This method demonstrates the Factory Method pattern -
     * it creates complex objects (VAMP environments) from simple configuration data.
     * The obstacle creation is type-safe and extensible to new obstacle types.
     */
    auto create_environment() -> vamp::collision::Environment<float> override
    {
        vamp::collision::Environment<float> vampEnvironment;
        
        for (const auto& obstacleConfig : m_obstacleConfigurations) {
            if (obstacleConfig.type == "sphere") {
                vampEnvironment.spheres.emplace_back(
                    vamp::collision::factory::sphere::array(obstacleConfig.position, obstacleConfig.radius)
                );
                if (!obstacleConfig.name.empty()) {
                    vampEnvironment.spheres.back().name = obstacleConfig.name;
                }
                
            } else if (obstacleConfig.type == "cuboid") {
                vampEnvironment.cuboids.emplace_back(
                    vamp::collision::factory::cuboid::array(
                        obstacleConfig.position, 
                        obstacleConfig.orientation_euler_xyz, 
                        obstacleConfig.half_extents
                    )
                );
                if (!obstacleConfig.name.empty()) {
                    vampEnvironment.cuboids.back().name = obstacleConfig.name;
                }
                
            } else if (obstacleConfig.type == "capsule") {
                vampEnvironment.capsules.emplace_back(
                    vamp::collision::factory::capsule::center::array(
                        obstacleConfig.position,
                        obstacleConfig.orientation_euler_xyz,
                        obstacleConfig.radius,
                        obstacleConfig.length
                    )
                );
                if (!obstacleConfig.name.empty()) {
                    vampEnvironment.capsules.back().name = obstacleConfig.name;
                }
            } else if (obstacleConfig.type == "pointcloud") {
                try {
                    // Resolve pointcloud file path using FileLocator
                    std::string resolvedPointcloudPath = FileLocator::findPointcloudFile(obstacleConfig.pointcloud_file);
                    auto points = PointcloudLoader::loadPointcloud(resolvedPointcloudPath);
                    if (!points.empty()) {
                        // Use default radii for CAPT construction (can be overridden by point_radius)
                        float r_min = 0.01f;  // Default minimum radius
                        float r_max = 1.0f;   // Default maximum radius
                        vampEnvironment.pointclouds.emplace_back(points, r_min, r_max, obstacleConfig.point_radius);
                        std::cout << "Loaded pointcloud: " << points.size() << " points from " 
                                  << resolvedPointcloudPath << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error loading pointcloud " << obstacleConfig.pointcloud_file 
                              << ": " << e.what() << std::endl;
                }
            } else {
                std::cerr << "Warning: Unknown obstacle type '" << obstacleConfig.type 
                          << "'. Skipping obstacle." << std::endl;
            }
        }
        
        vampEnvironment.sort();
        return vampEnvironment;
    }
    
    auto get_environment_name() const -> std::string override
    {
        return m_environmentName;
    }
    
    auto get_description() const -> std::string override
    {
        if (m_obstacleConfigurations.empty()) {
            return m_environmentDescription + " (empty environment)";
        }
        
        std::string fullDescription = m_environmentDescription + " (" + std::to_string(m_obstacleConfigurations.size()) + " obstacles: ";
        std::map<std::string, int> obstacleTypeCounts;
        for (const auto& obstacleConfig : m_obstacleConfigurations) {
            obstacleTypeCounts[obstacleConfig.type]++;
        }
        bool isFirstType = true;
        for (const auto& [obstacleType, obstacleCount] : obstacleTypeCounts) {
            if (!isFirstType) fullDescription += ", ";
            fullDescription += std::to_string(obstacleCount) + " " + obstacleType + (obstacleCount > 1 ? "s" : "");
            isFirstType = false;
        }
        fullDescription += ")";
        return fullDescription;
    }
    
    /**
     * @brief Get obstacle configurations (for sharing with Python visualization)
     */
    const std::vector<ObstacleConfig>& getObstacles() const {
        return m_obstacleConfigurations;
    }
};

/**
 * @brief Output utilities for VAMP-OMPL demos with  formatting
 * 
 * 
 *  Benefits:
 * - Consistent formatting across all demo applications
 * - Centralized presentation logic for easy maintenance
 * - Template-based design for type-safe robot information display
 * - Clear separation of concerns from core planning functionality
 */
class OutputFormatter {
public:
    /**
     * @brief Print concise usage information
     */
    static void printUsage(const char* programName) {
        std::cout << "\n VAMP-OMPL Demo (SIMD-accelerated motion planning)\n"
                  << "Usage: " << programName << " [--visualize | --help | config.yaml]\n"
                  << "  (no args)     - Basic programmatic example\n"
                  << "  --visualize   - Basic example + visualization\n"  
                  << "  config.yaml   - Use YAML configuration\n" << std::endl;
    }
    
    /**
     * @brief Print configuration summary in compact format
     */
    static void printConfigSummary(const PlanningConfiguration& planningConfiguration) {
        std::cout << " Config: " << planningConfiguration.robot_name << " + " << planningConfiguration.planning.planner_name 
                  << " (" << planningConfiguration.planning.planning_time << "s, " << planningConfiguration.start_config.size() 
                  << " joints, " << planningConfiguration.obstacles.size() << " obstacles)" << std::endl;
    }
    
    /**
     * @brief Print planning results in compact format  
     */
    static void printResults(const MotionPlanningResult& motionPlanningResult) {
        if (motionPlanningResult.success()) {
            std::cout << "\n Success: " << motionPlanningResult.planning_time_us() << "μs, " 
                      << motionPlanningResult.path_length() << " states, cost=" << motionPlanningResult.final_cost() << std::endl;
            if (!motionPlanningResult.solution_file_path.empty()) {
                std::cout << "  Solution saved: " << motionPlanningResult.solution_file_path << std::endl;
            }
        } else {
            std::cout << "\n Planning failed: " << motionPlanningResult.error_message() << std::endl;
        }
    }
    
    /**
     * @brief Print planner configuration summary with  details
     * @tparam Robot VAMP robot type
     * @param robotConfiguration Robot configuration
     * @param environmentFactory Environment factory
     * @param isInitialized Whether planner is initialized
     * 
     *  Note: This template function demonstrates how to provide
     * type-safe, compile-time polymorphism for different robot types while
     * maintaining a clean, unified interface for display purposes.
     */
    template<typename Robot>
    static void printPlannerConfiguration(const RobotConfig<Robot>& robotConfiguration, 
                                        const EnvironmentFactory& environmentFactory,
                                        bool isInitialized) {
        constexpr size_t robotDimension = Robot::dimension;
        std::cout << "\n=== VAMP-OMPL Planner Configuration ===\n"
                  << "Robot: " << robotConfiguration.getRobotName() << " (dim=" << robotDimension << ")\n"
                  << "Environment: " << environmentFactory.getEnvironmentName() << "\n"
                  << "Description: " << environmentFactory.getDescription() << "\n"
                  << "Initialized: " << (isInitialized ? "Yes" : "No") << std::endl;
        
        if (isInitialized) {
            auto robotJointLimits = robotConfiguration.getJointLimits();
            std::cout << "Joint Limits: ";
            for (size_t jointIndex = 0; jointIndex < robotJointLimits.size(); ++jointIndex) {
                std::cout << "[" << robotJointLimits[jointIndex].first << "," << robotJointLimits[jointIndex].second << "]";
                if (jointIndex < robotJointLimits.size() - 1) std::cout << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::string(40, '=') << std::endl;
    }
};





/**
 * @brief Visualization launcher for completed planning results
 * 
 * This class demonstrates the Command pattern for launching external visualization
 * processes. It encapsulates the complexity of cross-platform process execution
 * and provides a clean interface for visualization management.
 * 
 */
class VisualizationLauncher {
public:
    /**
     * @brief Resolve YAML config file path with search logic
     */
    static std::filesystem::path resolveYamlConfigPath(const std::string& yamlFileName) {
        // Search paths for YAML config files (same logic as Python script)
        std::vector<std::filesystem::path> searchPaths = {
            yamlFileName,                                               // Direct path
            std::filesystem::path("config") / yamlFileName,             // config/ directory
            std::filesystem::path("demos/Vamp/config") / yamlFileName,  // From project root
            std::filesystem::path("../demos/Vamp/config") / yamlFileName,      // From build dir
            std::filesystem::path("../../demos/Vamp/config") / yamlFileName    // From deeper dirs
        };
        
        // Try each path
        for (const auto& path : searchPaths) {
            if (std::filesystem::exists(path)) {
                return std::filesystem::absolute(path);  // Return absolute path
            }
        }
        
        // Return original if not found (will be handled by caller)
        return yamlFileName;
    }

    /**
     * @brief Run visualization for a completed planning result
     * @param planningResult Planning result containing solution path
     * @param configurationSource Configuration source (YAML filename or "programmatic")
     * @return true if visualization succeeded, false otherwise
     * 
     *  Note: This method demonstrates how to integrate external tools
     * into a C++ application while maintaining proper error handling and user feedback.
     */
    static bool runVisualization(const MotionPlanningResult& motionPlanningResult, const std::string& configurationSource) {
        if (!motionPlanningResult.success() || motionPlanningResult.solution_file_path.empty()) {
            std::cout << " No solution path to visualize" << std::endl;
            return false;
        }
        
        // Find visualization script relative to this source file location
        // This ensures the script is found regardless of working directory
        std::filesystem::path visualizationScriptPath = 
            std::filesystem::path(__FILE__).parent_path() / "visualization" / "visualize_solution.py";
        
        if (!std::filesystem::exists(visualizationScriptPath)) {
            std::cout << " Visualization script not found at: " << visualizationScriptPath << std::endl;
            return false;
        }
        
        // Use actual robot name from result, fallback to "panda" if not available
        std::string robotName = motionPlanningResult.robot_name.empty() ? "panda" : motionPlanningResult.robot_name;
        
        // Build visualization command
        std::string visualizationCommand;
        std::string scriptPathStr = visualizationScriptPath.string();
        
        // Use embedded visualization configuration (no need for separate robot/YAML args)
        visualizationCommand = "python3 " + scriptPathStr + " \"" + motionPlanningResult.solution_file_path + "\"";
        
        // Add optional YAML config if explicitly provided
        if (configurationSource.size() >= 5 && configurationSource.substr(configurationSource.size() - 5) == ".yaml") {
            // Resolve the full path to the YAML config file
            std::filesystem::path yamlConfigPath = resolveYamlConfigPath(configurationSource);
            if (std::filesystem::exists(yamlConfigPath)) {
                visualizationCommand += " --yaml-config \"" + yamlConfigPath.string() + "\"";
            } else {
                std::cout << "Warning: YAML config file not found: " << configurationSource << std::endl;
                std::cout << "Continuing without environment configuration..." << std::endl;
            }
        }
        
        std::cout << " Running visualization..." << std::endl;
        std::cout << "Command: " << visualizationCommand << std::endl;
        return std::system(visualizationCommand.c_str()) == 0;
    }
};

/**
 * @brief YAML configuration loader using yaml-cpp for robust parsing
 * 
 *  Design Features:
 * - Separation of parsing logic from business logic
 * - Robust error handling with actionable error messages
 * - Extensible parsing framework for new configuration sections
 * - Type-safe conversions with comprehensive validation
 */
class YamlConfigLoader {
private:
    /**
     * @brief Safe conversion from YAML node to specific types with  error handling
     * 
     */
    template<typename TargetType>
    static TargetType safeConvertYamlValue(const YAML::Node& yamlNode, const TargetType& defaultValue = TargetType{}) {
        try {
            return yamlNode.as<TargetType>();
        } catch (const YAML::Exception& yamlException) {
            std::cerr << "Warning: YAML conversion failed, using default. Error: " << yamlException.what() << std::endl;
            return defaultValue;
        }
    }
    
    /**
     * @brief Parse robot section from YAML
     */
    static void parseRobotSection(const YAML::Node& robotNode, PlanningConfiguration& planningConfiguration) {
        if (!robotNode) return;
        
        planningConfiguration.robot_name = safeConvertYamlValue<std::string>(robotNode["name"]);
        planningConfiguration.description = safeConvertYamlValue<std::string>(robotNode["description"]);
        
        // Note: Robot validation is performed at runtime by the registry system
    }
    
    /**
     * @brief Parse planner section from YAML
     */
    static void parsePlannerSection(const YAML::Node& plannerNode, PlanningConfiguration& planningConfiguration) {
        if (!plannerNode) return;
        
        planningConfiguration.planning.planner_name = safeConvertYamlValue<std::string>(plannerNode["name"], "BIT*");
        planningConfiguration.planning.planning_time = safeConvertYamlValue<double>(plannerNode["planning_time"], constants::DEFAULT_PLANNING_TIME);
        planningConfiguration.planning.simplification_time = safeConvertYamlValue<double>(plannerNode["simplification_time"], constants::DEFAULT_SIMPLIFICATION_TIME);
        planningConfiguration.planning.optimize_path = safeConvertYamlValue<bool>(plannerNode["optimize_path"], false);
        
        // Parse planner parameters (optional)
        if (plannerNode["parameters"]) {
            const YAML::Node& parametersNode = plannerNode["parameters"];
            for (auto it = parametersNode.begin(); it != parametersNode.end(); ++it) {
                std::string paramName = it->first.as<std::string>();
                std::string paramValue = safeConvertYamlValue<std::string>(it->second);
                planningConfiguration.planning.planner_parameters[paramName] = paramValue;
            }
        }
    }
    
    /**
     * @brief Parse single obstacle from YAML node
     */
    static ObstacleConfig parseObstacle(const YAML::Node& obstacleNode) {
        ObstacleConfig parsedObstacle;
        
        parsedObstacle.type = safeConvertYamlValue<std::string>(obstacleNode["type"]);
        parsedObstacle.name = safeConvertYamlValue<std::string>(obstacleNode["name"]);
        
        // Parse type-specific parameters only if they exist
        if (obstacleNode["radius"]) {
            parsedObstacle.radius = safeConvertYamlValue<float>(obstacleNode["radius"], constants::DEFAULT_SPHERE_RADIUS);
        }
        if (obstacleNode["length"]) {
            parsedObstacle.length = safeConvertYamlValue<float>(obstacleNode["length"], constants::DEFAULT_CAPSULE_LENGTH);
        }
        
        // Parse position array
        if (obstacleNode["position"] && obstacleNode["position"].IsSequence()) {
            auto positionSequence = obstacleNode["position"];
            for (size_t coordinateIndex = 0; coordinateIndex < std::min(positionSequence.size(), parsedObstacle.position.size()); ++coordinateIndex) {
                parsedObstacle.position[coordinateIndex] = safeConvertYamlValue<float>(positionSequence[coordinateIndex], 0.0f);
            }
        }
        
        // Parse orientation array
        if (obstacleNode["orientation"] && obstacleNode["orientation"].IsSequence()) {
            auto orientationSequence = obstacleNode["orientation"];
            for (size_t angleIndex = 0; angleIndex < std::min(orientationSequence.size(), parsedObstacle.orientation_euler_xyz.size()); ++angleIndex) {
                parsedObstacle.orientation_euler_xyz[angleIndex] = safeConvertYamlValue<float>(orientationSequence[angleIndex], 0.0f);
            }
        }
        
        // Parse half_extents array
        if (obstacleNode["half_extents"] && obstacleNode["half_extents"].IsSequence()) {
            auto extentsSequence = obstacleNode["half_extents"];
            for (size_t dimensionIndex = 0; dimensionIndex < std::min(extentsSequence.size(), parsedObstacle.half_extents.size()); ++dimensionIndex) {
                parsedObstacle.half_extents[dimensionIndex] = safeConvertYamlValue<float>(extentsSequence[dimensionIndex], constants::DEFAULT_HALF_EXTENTS[dimensionIndex]);
            }
        }
        
        // Parse pointcloud-specific parameters
        if (obstacleNode["pointcloud_file"]) {
            parsedObstacle.pointcloud_file = safeConvertYamlValue<std::string>(obstacleNode["pointcloud_file"]);
        }
        if (obstacleNode["point_radius"]) {
            parsedObstacle.point_radius = safeConvertYamlValue<float>(obstacleNode["point_radius"], constants::DEFAULT_POINT_RADIUS);
        }
        
        return parsedObstacle;
    }
    
    /**
     * @brief Parse obstacles section
     */
    static void parseObstaclesSection(const YAML::Node& obstaclesNode, PlanningConfiguration& planningConfiguration) {
        planningConfiguration.obstacles.clear();
        
        if (!obstaclesNode || !obstaclesNode.IsSequence()) return;
        
        for (const auto& obstacleNode : obstaclesNode) {
            try {
                planningConfiguration.obstacles.push_back(parseObstacle(obstacleNode));
            } catch (const std::exception& parseException) {
                std::cerr << "Warning: Failed to parse obstacle, skipping. Error: " << parseException.what() << std::endl;
            }
        }
    }
    
    /**
     * @brief Parse output section
     */
    static void parseOutputSection(const YAML::Node& outputNode, PlanningConfiguration& planningConfiguration) {
        if (!outputNode) return;
        
        planningConfiguration.save_path = safeConvertYamlValue<bool>(outputNode["write_path"], false);
        if (outputNode["description"]) {
            planningConfiguration.description = safeConvertYamlValue<std::string>(outputNode["description"]);
        }
    }
    
    /**
     * @brief Parse visualization section
     */
    static void parseVisualizationSection(const YAML::Node& visualizationNode, PlanningConfiguration& planningConfiguration) {
        if (!visualizationNode) return;
        
        // Parse URDF path
        if (visualizationNode["urdf_path"]) {
            planningConfiguration.visualization.urdf_path = safeConvertYamlValue<std::string>(visualizationNode["urdf_path"]);
        }
        
        // Parse expected joints
        if (visualizationNode["expected_joints"]) {
            planningConfiguration.visualization.expected_joints = safeConvertYamlValue<int>(visualizationNode["expected_joints"], -1);
        }
        
        // Parse base position
        if (visualizationNode["base_position"] && visualizationNode["base_position"].IsSequence()) {
            auto positionSequence = visualizationNode["base_position"];
            for (size_t i = 0; i < std::min(positionSequence.size(), planningConfiguration.visualization.base_position.size()); ++i) {
                planningConfiguration.visualization.base_position[i] = safeConvertYamlValue<float>(positionSequence[i], 0.0f);
            }
        }
        
        // Parse base orientation
        if (visualizationNode["base_orientation"] && visualizationNode["base_orientation"].IsSequence()) {
            auto orientationSequence = visualizationNode["base_orientation"];
            for (size_t i = 0; i < std::min(orientationSequence.size(), planningConfiguration.visualization.base_orientation.size()); ++i) {
                planningConfiguration.visualization.base_orientation[i] = safeConvertYamlValue<float>(orientationSequence[i], 0.0f);
            }
        }
        
        // Parse other parameters
        if (visualizationNode["use_fixed_base"]) {
            planningConfiguration.visualization.use_fixed_base = safeConvertYamlValue<bool>(visualizationNode["use_fixed_base"], true);
        }
        
        if (visualizationNode["description"]) {
            planningConfiguration.visualization.description = safeConvertYamlValue<std::string>(visualizationNode["description"]);
        }
    }
    
    /**
     * @brief Parse configuration arrays (start_config, goal_config)
     */
    static std::vector<float> parseConfigurationArray(const YAML::Node& configurationArrayNode) {
        std::vector<float> parsedConfiguration;
        
        if (!configurationArrayNode || !configurationArrayNode.IsSequence()) return parsedConfiguration;
        
        parsedConfiguration.reserve(configurationArrayNode.size());
        for (const auto& configurationItem : configurationArrayNode) {
            parsedConfiguration.push_back(safeConvertYamlValue<float>(configurationItem, 0.0f));
        }
        
        return parsedConfiguration;
    }
    
public:
    /**
     * @brief Main YAML configuration loader using yaml-cpp
     * 
     */
    static bool loadYamlConfig(const std::string& yamlFilename, PlanningConfiguration& planningConfiguration) {
        try {
            std::string resolvedFilePath = FileLocator::findYamlFile(yamlFilename);
            std::cout << "Loading: " << resolvedFilePath << std::endl;
            
            YAML::Node yamlConfiguration = YAML::LoadFile(resolvedFilePath);
            
            // Parse each section
            parseRobotSection(yamlConfiguration["robot"], planningConfiguration);
            parsePlannerSection(yamlConfiguration["planner"], planningConfiguration);
            
            // Parse configuration arrays
            planningConfiguration.start_config = parseConfigurationArray(yamlConfiguration["start_config"]);
            planningConfiguration.goal_config = parseConfigurationArray(yamlConfiguration["goal_config"]);
            
            // Parse obstacles and output
            parseObstaclesSection(yamlConfiguration["obstacles"], planningConfiguration);
            parseOutputSection(yamlConfiguration["output"], planningConfiguration);
            parseVisualizationSection(yamlConfiguration["visualization"], planningConfiguration);
            
            // Validate configuration
            if (!planningConfiguration.isValid()) {
                throw VampYamlError("Invalid configuration: " + planningConfiguration.getValidationErrors());
            }
            
            std::cout << " YAML loaded: " << planningConfiguration.robot_name << " + " << planningConfiguration.planning.planner_name 
                      << " (" << planningConfiguration.start_config.size() << " joints, " << planningConfiguration.obstacles.size() << " obstacles)" << std::endl;
            
            return true;
            
        } catch (const YAML::Exception& yamlException) {
            std::cerr << " YAML parsing failed: " << yamlException.what() << std::endl;
            return false;
        } catch (const std::exception& generalException) {
            std::cerr << " YAML loading failed: " << generalException.what() << std::endl;
            return false;
        }
    }
};

/**
 * @brief Common utilities for VAMP-OMPL demos with  organization
 * 
 * This class consolidates essential functionality for VAMP integration demos.
 * Simplified and focused on core responsibilities.
 * 
 * Benefits:
 * - Clean, consistent interface for client code
 * - Easy to extend with new utility functions
 * - Maintains backward compatibility
 * - Promotes code reuse across different demo applications
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
     * @param config Configuration vector to validate
     * @param expected_dimension Expected robot dimension
     * @param config_type Configuration type (e.g., "start", "goal") for error messages
     * @param robot_name Robot name for error messages
     * @throws VampConfigurationError if validation fails
     */
    static void validateConfigurationDimension(
        const std::vector<float>& config,
        std::size_t expected_dimension,
        const std::string& config_type,
        const std::string& robot_name) {
        
        if (config.size() != expected_dimension) {
            throw VampConfigurationError(config_type + " configuration dimension (" + 
                std::to_string(config.size()) + ") does not match robot dimension (" + 
                std::to_string(expected_dimension) + ") for " + robot_name);
        }
    }
    
    /**
     * @brief Validate configuration values against joint limits
     * @param config Configuration vector to validate
     * @param limits Joint limits as pairs of (min, max)
     * @param config_type Configuration type for error messages
     * @param robot_name Robot name for error messages
     * @throws VampConfigurationError if any joint is outside limits
     */
    static void validateConfigurationLimits(
        const std::vector<float>& config,
        const std::vector<std::pair<double, double>>& limits,
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

    // ========== DELEGATE METHODS ==========
    // Delegate to specialized classes for better organization
    static void printUsage(const char* programName) {
        OutputFormatter::printUsage(programName);
    }
    
    static void printConfigSummary(const PlanningConfiguration& planningConfiguration) {
        OutputFormatter::printConfigSummary(planningConfiguration);
    }
    
    static void printResults(const MotionPlanningResult& motionPlanningResult) {
        OutputFormatter::printResults(motionPlanningResult);
    }
    
    template<typename Robot>
    static void printPlannerConfiguration(const RobotConfig<Robot>& robotConfiguration, 
                                        const EnvironmentFactory& environmentFactory,
                                        bool isInitialized) {
        OutputFormatter::printPlannerConfiguration<Robot>(robotConfiguration, environmentFactory, isInitialized);
    }
    
    static bool runVisualization(const MotionPlanningResult& motionPlanningResult, const std::string& configurationSource) {
        return VisualizationLauncher::runVisualization(motionPlanningResult, configurationSource);
    }
    
    static std::string findYamlFile(const std::string& yamlFilename) {
        return FileLocator::findYamlFile(yamlFilename);
    }
    
    static bool loadYamlConfig(const std::string& yamlFilename, PlanningConfiguration& planningConfiguration) {
        return YamlConfigLoader::loadYamlConfig(yamlFilename, planningConfiguration);
    }
    
    // Backward compatibility - maintain existing interface
    static std::string findFile(const std::string& targetFilename, const std::vector<std::string>& searchPaths) {
        return FileLocator::findFile(targetFilename, searchPaths);
    }
};

} // namespace vamp_ompl 