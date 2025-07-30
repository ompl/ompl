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
    constexpr double DEFAULT_SIMPLIFICATION_TIME = 0.5;
    constexpr float DEFAULT_SPHERE_RADIUS = 0.1f;
    constexpr float DEFAULT_CAPSULE_LENGTH = 0.2f;
    constexpr std::array<float, 3> DEFAULT_HALF_EXTENTS = {0.1f, 0.1f, 0.1f};
    
    // Supported robot types
    inline std::vector<std::string> getSupportedRobots() {
        return {"panda", "ur5", "fetch"};
    }
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
 * @brief Single configurable environment factory (SOLID principles)
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
    
public:
    /**
     * @brief Constructor with explicit obstacles list
     */
    ConfigurableEnvironmentFactory(const std::vector<ObstacleConfig>& obstacleConfigurations = {},
                                 const std::string& environmentName = "Environment",
                                 const std::string& environmentDescription = "Explicit obstacle environment")
        : m_obstacleConfigurations(obstacleConfigurations), m_environmentName(environmentName), m_environmentDescription(environmentDescription)
    {
    }
    
    /**
     * @brief Set obstacles for this environment (explicit configuration required)
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
     * @brief Create VAMP environment from explicit obstacle configurations
     * 
     *  Note: This method demonstrates the Factory Method pattern -
     * it creates complex objects (VAMP environments) from simple configuration data.
     * The obstacle creation is type-safe and extensible to new obstacle types.
     */
    vamp::collision::Environment<float> createEnvironment() override
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
            } else {
                std::cerr << "Warning: Unknown obstacle type '" << obstacleConfig.type 
                          << "'. Skipping obstacle." << std::endl;
            }
        }
        
        vampEnvironment.sort();
        return vampEnvironment;
    }
    
    std::string getEnvironmentName() const override
    {
        return m_environmentName;
    }
    
    std::string getDescription() const override
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
    
    /**
     * @brief Set environment name and description
     */
    void setMetadata(const std::string& environmentName, const std::string& environmentDescription) {
        m_environmentName = environmentName;
        m_environmentDescription = environmentDescription;
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
        std::cout << "\n🎯 VAMP-OMPL Demo (SIMD-accelerated motion planning)\n"
                  << "Usage: " << programName << " [--visualize | --help | config.yaml]\n"
                  << "  (no args)     - Basic programmatic example\n"
                  << "  --visualize   - Basic example + visualization\n"  
                  << "  config.yaml   - Use YAML configuration\n" << std::endl;
    }
    
    /**
     * @brief Print configuration summary in compact format
     */
    static void printConfigSummary(const PlanningConfiguration& planningConfiguration) {
        std::cout << "📋 Config: " << planningConfiguration.robot_name << " + " << planningConfiguration.planning.planner_name 
                  << " (" << planningConfiguration.planning.planning_time << "s, " << planningConfiguration.start_config.size() 
                  << " joints, " << planningConfiguration.obstacles.size() << " obstacles)" << std::endl;
    }
    
    /**
     * @brief Print planning results in compact format  
     */
    static void printResults(const MotionPlanningResult& motionPlanningResult) {
        if (motionPlanningResult.success()) {
            std::cout << "\n✅ Success: " << motionPlanningResult.planning_time_us() << "μs, " 
                      << motionPlanningResult.path_length() << " states, cost=" << motionPlanningResult.final_cost() << std::endl;
            if (!motionPlanningResult.solution_file_path.empty()) {
                std::cout << "  Solution saved: " << motionPlanningResult.solution_file_path << std::endl;
            }
        } else {
            std::cout << "\n❌ Planning failed: " << motionPlanningResult.error_message() << std::endl;
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
            if (testFileStream.is_open()) return candidatePath;
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
            std::cout << "❌ No solution path to visualize" << std::endl;
            return false;
        }
        
        // Find visualization script in common locations
        std::vector<std::string> visualizationScriptPaths = {
            "visualize_solution.py",
            "demos/Vamp/visualize_solution.py",
            "../demos/Vamp/visualize_solution.py", 
            "../../demos/Vamp/visualize_solution.py"
        };
        
        std::string visualizationScriptPath;
        for (const auto& candidatePath : visualizationScriptPaths) {
            if (std::filesystem::exists(candidatePath)) {
                visualizationScriptPath = candidatePath;
                break;
            }
        }
        
        if (visualizationScriptPath.empty()) {
            std::cout << "❌ Visualization script not found" << std::endl;
            return false;
        }
        
        // Use actual robot name from result, fallback to "panda" if not available
        std::string robotName = motionPlanningResult.robot_name.empty() ? "panda" : motionPlanningResult.robot_name;
        
        // Build visualization command
        std::string visualizationCommand;
        if (configurationSource.size() >= 5 && configurationSource.substr(configurationSource.size() - 5) == ".yaml") {
            // Use YAML configuration for visualization
            visualizationCommand = "python3 " + visualizationScriptPath + 
                  " --robot " + robotName +
                  " --yaml-config " + configurationSource +
                  " \"" + motionPlanningResult.solution_file_path + "\"";
        } else {
            // Use the default YAML for programmatic example
            visualizationCommand = "python3 " + visualizationScriptPath + 
                  " --robot " + robotName +
                  " --yaml-config panda_demo.yaml" +
                  " \"" + motionPlanningResult.solution_file_path + "\"";
        }
        
        std::cout << "🎬 Running visualization..." << std::endl;
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
        
        // Validate robot type
        if (!planningConfiguration.robot_name.empty()) {
            const auto supportedRobots = constants::getSupportedRobots();
            if (std::find(supportedRobots.begin(), supportedRobots.end(), planningConfiguration.robot_name) == supportedRobots.end()) {
                std::string robotList;
                for (size_t i = 0; i < supportedRobots.size(); ++i) {
                    robotList += supportedRobots[i];
                    if (i < supportedRobots.size() - 1) robotList += ", ";
                }
                throw VampYamlError("Unsupported robot '" + planningConfiguration.robot_name + 
                                  "' in YAML. Supported robots: " + robotList);
            }
        }
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
    }
    
    /**
     * @brief Parse single obstacle from YAML node
     */
    static ObstacleConfig parseObstacle(const YAML::Node& obstacleNode) {
        ObstacleConfig parsedObstacle;
        
        parsedObstacle.type = safeConvertYamlValue<std::string>(obstacleNode["type"]);
        parsedObstacle.name = safeConvertYamlValue<std::string>(obstacleNode["name"]);
        parsedObstacle.radius = safeConvertYamlValue<float>(obstacleNode["radius"], constants::DEFAULT_SPHERE_RADIUS);
        parsedObstacle.length = safeConvertYamlValue<float>(obstacleNode["length"], constants::DEFAULT_CAPSULE_LENGTH);
        
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
            std::cout << "📁 Loading: " << resolvedFilePath << std::endl;
            
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
            
            // Validate configuration
            if (!planningConfiguration.isValid()) {
                throw VampYamlError("Invalid configuration: " + planningConfiguration.getValidationErrors());
            }
            
            std::cout << "✅ YAML loaded: " << planningConfiguration.robot_name << " + " << planningConfiguration.planning.planner_name 
                      << " (" << planningConfiguration.start_config.size() << " joints, " << planningConfiguration.obstacles.size() << " obstacles)" << std::endl;
            
            return true;
            
        } catch (const YAML::Exception& yamlException) {
            std::cerr << "❌ YAML parsing failed: " << yamlException.what() << std::endl;
            return false;
        } catch (const std::exception& generalException) {
            std::cerr << "❌ YAML loading failed: " << generalException.what() << std::endl;
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
    
    /**
     * @brief Create example YAML configuration for a robot
     * @param robotName Name of the robot (panda, ur5, fetch)
     * @param outputFilename Output YAML filename
     * @return true if file was created successfully
     */
    static bool createExampleYamlConfig(const std::string& robotName, const std::string& outputFilename) {
        // Validate robot type
        const auto supportedRobots = constants::getSupportedRobots();
        if (std::find(supportedRobots.begin(), supportedRobots.end(), robotName) == supportedRobots.end()) {
            std::cout << "❌ Unsupported robot: " << robotName << std::endl;
            return false;
        }
        
        try {
            std::ofstream yamlFile(outputFilename);
            if (!yamlFile.is_open()) {
                std::cout << "❌ Could not create file: " << outputFilename << std::endl;
                return false;
            }
            
            yamlFile << "# Example " << robotName << " Demo Configuration\n";
            yamlFile << "# Generated by VAMP-OMPL integration demo\n\n";
            
            yamlFile << "# Robot Configuration\n";
            yamlFile << "robot:\n";
            yamlFile << "  name: \"" << robotName << "\"\n";
            yamlFile << "  description: \"Example " << robotName << " configuration\"\n\n";
            
            yamlFile << "# Planning Configuration\n";
            yamlFile << "planner:\n";
            yamlFile << "  name: \"RRT-Connect\"\n";
            yamlFile << "  planning_time: 2.0\n";
            yamlFile << "  simplification_time: 1.0\n";
            yamlFile << "  optimize_path: false\n\n";
            
            yamlFile << "# Start Configuration (modify for your specific scenario)\n";
            yamlFile << "start_config:\n";
            int dof = (robotName == "panda") ? 7 : (robotName == "ur5") ? 6 : 8; // Approximate DOF
            for (int i = 0; i < dof; ++i) {
                yamlFile << "  - 0.0  # joint" << (i+1) << "\n";
            }
            
            yamlFile << "\n# Goal Configuration (modify for your specific scenario)\n";
            yamlFile << "goal_config:\n";
            for (int i = 0; i < dof; ++i) {
                yamlFile << "  - " << (i == 0 ? "1.0" : "0.0") << "  # joint" << (i+1) << "\n";
            }
            
            yamlFile << "\n# Environment Configuration\n";
            yamlFile << "obstacles:\n";
            yamlFile << "  # Example sphere obstacle\n";
            yamlFile << "  - type: sphere\n";
            yamlFile << "    name: \"example_sphere\"\n";
            yamlFile << "    position: [0.5, 0.0, 0.5]\n";
            yamlFile << "    radius: 0.1\n\n";
            
            yamlFile << "# Output Configuration\n";
            yamlFile << "output:\n";
            yamlFile << "  write_path: true\n";
            yamlFile << "  description: \"Example " << robotName << " demo\"\n";
            
            yamlFile.close();
            std::cout << "✓ Created example YAML config: " << outputFilename << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "❌ Error creating YAML config: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Backward compatibility - maintain existing interface
    static std::string findFile(const std::string& targetFilename, const std::vector<std::string>& searchPaths) {
        return FileLocator::findFile(targetFilename, searchPaths);
    }
};

} // namespace vamp_ompl 