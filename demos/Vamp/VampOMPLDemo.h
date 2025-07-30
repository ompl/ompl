/**
 * @file VampOMPLDemo.h
 * @brief High-level VAMP-OMPL integration demo framework
 * 
 * This file provides a complete demonstration framework for VAMP-OMPL integration,
 * showcasing vectorized motion planning with microsecond-level performance.
 * 
 * Key Features:
 * - Template-based robot support (Panda, UR5, Fetch)
 * - Flexible environment system with custom obstacle support
 * - Unified planning interface with multiple OMPL planners
 * - YAML-driven configuration for reproducible experiments
 * - Integrated path visualization capabilities
 * 
 * Architecture Overview:
 * The demo follows a layered architecture with clear separation of concerns:
 * 1. Configuration Layer: DemoConfiguration, YAML parsing
 * 2. Planning Layer: Robot configs, environment factories, OMPL context
 * 3. Execution Layer: Unified planning functions, result handling
 * 4. Visualization Layer: Path writing and PyBullet integration
 * 
 */
#pragma once

// Include all VAMP-OMPL integration components
#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include "OMPLPlanningContext.h"
#include "RobotConfigurations.h"
#include "EnvironmentFactories.h"
#include "VampOMPLPlanner.h"

// Standard library includes to fix compilation errors
#include <memory>
#include <thread>
#include <functional>
#include <tuple>

// VAMP robot includes
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>

namespace vamp_ompl {

/**
 * @brief Demo configuration that combines robot, environment, and planning settings
 */
struct DemoConfiguration {
    std::string robot_name;          ///< Name of robot ("panda", "ur5", "fetch")
    std::string environment_name;    ///< Name of environment ("empty", "sphere_cage", "table", "corridor")
    std::string planner_name;        ///< Name of planner ("BIT*", "RRT-Connect", "PRM")
    double planning_time;            ///< Planning time in seconds
    double simplification_time;      ///< Simplification time in seconds
    bool optimize_path;              ///< Whether to optimize path cost
    bool write_path;                 ///< Whether to write solution path to file
    std::string description;         ///< Description of this demo
    
    DemoConfiguration(const std::string& robot = "panda",
                     const std::string& env = "sphere_cage", 
                     const std::string& planner = "BIT*",
                     double plan_time = 1.0,
                     double simp_time = 0.5,
                     bool optimize = false,
                     bool write_path_to_file = false,
                     const std::string& desc = "VAMP-OMPL Demo")
        : robot_name(robot), environment_name(env), planner_name(planner),
          planning_time(plan_time), simplification_time(simp_time),
          optimize_path(optimize), write_path(write_path_to_file), description(desc)
    {
    }
};

/**
 * @brief Create robot configuration by name
 */
template<typename Robot>
std::unique_ptr<RobotConfig<Robot>> createRobotConfig(const std::string& robot_name, 
                                                     const std::string& env_name = "");

// Template specializations for robot config creation
template<>
inline std::unique_ptr<RobotConfig<vamp::robots::Panda>> 
createRobotConfig<vamp::robots::Panda>(const std::string& robot_name, const std::string& env_name)
{
    if (robot_name != "panda") {
        throw std::invalid_argument("Robot name must be 'panda' for Panda robot");
    }
    
    if (env_name == "table") {
        return std::make_unique<PandaTableConfig>();
    } else {
        return std::make_unique<PandaConfig>();
    }
}

template<>
inline std::unique_ptr<RobotConfig<vamp::robots::UR5>> 
createRobotConfig<vamp::robots::UR5>(const std::string& robot_name, const std::string& /* env_name */)
{
    if (robot_name != "ur5") {
        throw std::invalid_argument("Robot name must be 'ur5' for UR5 robot");
    }
    return std::make_unique<UR5Config>();
}

template<>
inline std::unique_ptr<RobotConfig<vamp::robots::Fetch>> 
createRobotConfig<vamp::robots::Fetch>(const std::string& robot_name, const std::string& /* env_name */)
{
    if (robot_name != "fetch") {
        throw std::invalid_argument("Robot name must be 'fetch' for Fetch robot");
    }
    return std::make_unique<FetchConfig>();
}

/**
 * @brief Parse obstacles from YAML configuration and create CustomEnvironmentFactory
 */
inline std::unique_ptr<CustomEnvironmentFactory> createCustomEnvironmentFromYaml(
    const std::vector<std::map<std::string, std::string>>& yaml_obstacles,
    const std::string& name = "Custom YAML Environment")
{
    auto factory = std::make_unique<CustomEnvironmentFactory>(
        std::vector<ObstacleConfig>{}, name, "Environment created from YAML configuration"
    );
    
    for (const auto& obstacle_map : yaml_obstacles) {
        try {
            std::string type = obstacle_map.at("type");
            
            // Parse position (required for all)
            std::array<float, 3> position;
            position[0] = std::stof(obstacle_map.at("position_x"));
            position[1] = std::stof(obstacle_map.at("position_y"));
            position[2] = std::stof(obstacle_map.at("position_z"));
            
            std::string obs_name = "";
            auto name_it = obstacle_map.find("name");
            if (name_it != obstacle_map.end()) {
                obs_name = name_it->second;
            }
            
            if (type == "sphere") {
                float radius = std::stof(obstacle_map.at("radius"));
                factory->addSphere(position, radius, obs_name);
                
            } else if (type == "cuboid") {
                std::array<float, 3> half_extents;
                half_extents[0] = std::stof(obstacle_map.at("half_extents_x"));
                half_extents[1] = std::stof(obstacle_map.at("half_extents_y"));
                half_extents[2] = std::stof(obstacle_map.at("half_extents_z"));
                
                std::array<float, 3> orientation = {0.0f, 0.0f, 0.0f};
                auto orient_x = obstacle_map.find("orientation_x");
                auto orient_y = obstacle_map.find("orientation_y");
                auto orient_z = obstacle_map.find("orientation_z");
                if (orient_x != obstacle_map.end()) orientation[0] = std::stof(orient_x->second);
                if (orient_y != obstacle_map.end()) orientation[1] = std::stof(orient_y->second);
                if (orient_z != obstacle_map.end()) orientation[2] = std::stof(orient_z->second);
                
                factory->addCuboid(position, half_extents, orientation, obs_name);
                
            } else if (type == "capsule") {
                float radius = std::stof(obstacle_map.at("radius"));
                float length = std::stof(obstacle_map.at("length"));
                
                std::array<float, 3> orientation;
                orientation[0] = std::stof(obstacle_map.at("orientation_x"));
                orientation[1] = std::stof(obstacle_map.at("orientation_y"));
                orientation[2] = std::stof(obstacle_map.at("orientation_z"));
                
                factory->addCapsule(position, orientation, radius, length, obs_name);
                
            } else {
                std::cerr << "Warning: Unknown obstacle type '" << type << "' in YAML. Skipping." << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error parsing obstacle from YAML: " << e.what() << std::endl;
        }
    }
    
    return factory;
}

/**
 * @brief Create environment factory by name
 */
inline std::unique_ptr<EnvironmentFactory> createEnvironmentFactory(const std::string& env_name)
{
    if (env_name == "empty") {
        return std::make_unique<EmptyEnvironmentFactory>();
    } else if (env_name == "sphere_cage") {
        return std::make_unique<SphereCageEnvironmentFactory>();
    } else if (env_name == "table") {
        return std::make_unique<TableSceneEnvironmentFactory>();
    } else if (env_name == "custom_mixed") {
        return CustomEnvironmentFactory::createSampleEnvironment("mixed");
    } else if (env_name == "custom_spheres") {
        return CustomEnvironmentFactory::createSampleEnvironment("spheres_only");
    } else if (env_name == "custom_cuboids") {
        return CustomEnvironmentFactory::createSampleEnvironment("cuboids_only");
    } else {
        throw std::invalid_argument("Unknown environment: " + env_name + 
                                  ". Available: empty, sphere_cage, table, custom_mixed, custom_spheres, custom_cuboids");
    }
}

/**
 * @brief Create planning configuration from demo configuration
 */
inline PlanningConfig createPlanningConfig(const DemoConfiguration& demo_config)
{
    return PlanningConfig(demo_config.planning_time, 
                         demo_config.simplification_time,
                         demo_config.optimize_path,
                         demo_config.planner_name,
                         demo_config.write_path);
}

/**
 * @brief Print planning results in a nice format
 */
inline void printPlanningResults(const DemoConfiguration& demo_config, 
                                const PlanningResult& result)
{
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "Planning Results: " << demo_config.description << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    std::cout << "Success: " << (result.success ? "✓" : "✗") << std::endl;
    
    if (result.success) {
        std::cout << "Planner Name: " << demo_config.planner_name << std::endl;
        std::cout << "Planning time: " << result.planning_time_us << " μs" << std::endl;
        std::cout << "Simplification time: " << result.simplification_time_us << " μs" << std::endl;
        std::cout << "Initial path cost: " << result.initial_cost << std::endl;
        std::cout << "Final path cost: " << result.final_cost << std::endl;
        std::cout << "Path length: " << result.path_length << " states" << std::endl;
        
        if (demo_config.write_path) {
            std::cout << "Path writing: ✓ Enabled (check for .txt file)" << std::endl;
        } else {
            std::cout << "Path writing: ✗ Disabled" << std::endl;
        }
        
        if (result.final_cost < result.initial_cost) {
            double improvement = (result.initial_cost - result.final_cost) / result.initial_cost * 100;
            std::cout << "Cost improvement: " << improvement << "%" << std::endl;
        }
    } else {
        std::cout << "Error: " << result.error_message << std::endl;
    }
    
    std::cout << std::string(60, '=') << std::endl;
}

/**
 * @brief Run a single demo with the specified configuration
 * @brief Unified planning execution interface
 * 
 * This function serves as the central planning execution point. It provides
 * a consistent interface for any robot type and environment combination.
 * 
 * 
 * @tparam Robot The robot type (e.g., vamp::robots::Panda)
 * @param demo_config Configuration specifying robot, environment, planner, and timing
 * @param env_factory Factory for creating the collision environment
 * @param execution_context Descriptive context for logging and error reporting
 * @return true if planning succeeded, false otherwise
 * 
 * @note This function handles the complete planning pipeline:
 *       1. Robot configuration creation and validation
 *       2. Planner initialization with VAMP validators
 *       3. Planning execution with timing measurement
 *       4. Result formatting and display
 */
template<typename Robot>
bool executePlanning(const DemoConfiguration& demo_config, 
                    std::unique_ptr<EnvironmentFactory> env_factory,
                    const std::string& execution_context = "Demo")
{
    try {
        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "Starting " << execution_context << ": " << demo_config.description << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        
        // Create robot configuration
        auto robot_config = createRobotConfig<Robot>(demo_config.robot_name, demo_config.environment_name);
        
        // Create planner
        auto planner = createVampOMPLPlanner(std::move(robot_config), std::move(env_factory));
        
        // Print configuration and initialize
        planner->printConfiguration();
        planner->initialize();
        
        // Create planning configuration and execute
        auto planning_config = createPlanningConfig(demo_config);
        std::cout << "\nStarting planning..." << std::endl;
        auto result = planner->plan(planning_config);
        
        // Print results
        printPlanningResults(demo_config, result);
        
        return result.success;
        
    } catch (const std::exception& e) {
        std::cout << "✗ " << execution_context << " failed: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Type-safe robot dispatch mechanism using template metaprogramming
 * 
 * This function implements a compile-time type dispatch pattern that eliminates
 * runtime type checking overhead. It allows the same planning logic to work
 * with different robot types while maintaining full type safety.
 * 
 * Implementation Pattern:
 * The function accepts a generic callable (lambda, function object, or function pointer)
 * and invokes it with the appropriate robot type as a template parameter. This
 * enables the compiler to generate optimized code paths for each robot type.
 * 
 * Usage Example:
 * @code
 * dispatchByRobotType("panda", []<typename Robot>() {
 *     return runSingleDemo<Robot>(config);
 * });
 * @endcode
 * 
 * @tparam Func Callable type that accepts a robot type as template parameter
 * @param robot_name String identifier for the robot type ("panda", "ur5", "fetch")
 * @param func Callable to invoke with the appropriate robot type
 * @return Result of the callable function, or false if robot type is unsupported
 * 
 * @note This pattern avoids dynamic_cast and virtual dispatch overhead while
 *       providing extensibility for new robot types through template specialization
 */
template<typename Func>
bool dispatchByRobotType(const std::string& robot_name, Func&& func)
{
    if (robot_name == "panda") {
        return func.template operator()<vamp::robots::Panda>();
    } else if (robot_name == "ur5") {
        return func.template operator()<vamp::robots::UR5>();
    } else if (robot_name == "fetch") {
        return func.template operator()<vamp::robots::Fetch>();
    } else {
        std::cout << "❌ Unsupported robot: " << robot_name << 
                     ". Available: panda, ur5, fetch" << std::endl;
        return false;
    }
}

/**
 * @brief Unified environment factory creation with support for all environment types
 * 
 * This factory function provides a single entry point for creating any type of
 * environment, whether predefined or custom. It abstracts the complexity of
 * different environment creation patterns behind a uniform interface.
 * 
 * Supported Environment Types:
 * - Standard environments: "empty", "sphere_cage", "table", "custom_mixed", etc.
 * - YAML-defined custom environments with obstacle specifications
 * - Programmatically-defined custom environments with ObstacleConfig vectors
 * - Empty custom environments for testing
 * 
 * Priority Order:
 * 1. Custom environments (yaml_obstacles or custom_obstacles provided)
 * 2. Named custom environments (env_name starts with "custom")
 * 3. Standard predefined environments
 * 
 * @param env_name Name of the environment to create
 * @param yaml_obstacles Optional YAML-parsed obstacle specifications
 * @param custom_obstacles Optional programmatically-defined obstacles
 * @param custom_env_name Name for custom environments (used in logging/visualization)
 * @return Unique pointer to the appropriate EnvironmentFactory implementation
 * 
 * @throws std::invalid_argument if env_name is not recognized and no custom data provided
 */
inline std::unique_ptr<EnvironmentFactory> createUnifiedEnvironmentFactory(
    const std::string& env_name,
    const std::vector<std::map<std::string, std::string>>* yaml_obstacles = nullptr,
    const std::vector<ObstacleConfig>* custom_obstacles = nullptr,
    const std::string& custom_env_name = "Custom Environment")
{
    // Handle custom environments first
    if (env_name == "custom" || yaml_obstacles || custom_obstacles) {
        if (yaml_obstacles && !yaml_obstacles->empty()) {
            return createCustomEnvironmentFromYaml(*yaml_obstacles, custom_env_name);
        } else if (custom_obstacles && !custom_obstacles->empty()) {
            return std::make_unique<CustomEnvironmentFactory>(*custom_obstacles, custom_env_name, 
                                                            "Custom environment with user-defined obstacles");
        } else {
            // Empty custom environment
            return std::make_unique<CustomEnvironmentFactory>(std::vector<ObstacleConfig>{}, 
                                                            "Empty Custom Environment", 
                                                            "Empty custom environment");
        }
    }
    
    // Handle standard environments
    return createEnvironmentFactory(env_name);
}

/**
 * @brief Get predefined demo configurations
 */
inline std::vector<DemoConfiguration> getPredefinedDemos()
{
    return {
        DemoConfiguration("panda", "sphere_cage", "RRT-Connect", 1.0, 0.5, false, false, 
                         "Panda in Sphere Cage with RRT-Connect"),
        DemoConfiguration("ur5", "sphere_cage", "PRM", 1.0, 0.5, false, false,
                         "UR5 in Sphere Cage with PRM"),
        DemoConfiguration("fetch", "empty", "RRT-Connect", 1.0, 0.5, false, false,
                         "Fetch in Empty Environment with RRT-Connect"),
        DemoConfiguration("panda", "table", "RRT-Connect", 1.0, 0.5, false, true,
                         "Panda in Table Scene with RRT-Connect (with path saving)")
    };
}

/**
 * @brief Run a single demo with the specified configuration (simplified version)
 */
template<typename Robot>
bool runSingleDemo(const DemoConfiguration& demo_config)
{
    auto env_factory = createUnifiedEnvironmentFactory(demo_config.environment_name);
    return executePlanning<Robot>(demo_config, std::move(env_factory), "Single Demo");
}

/**
 * @brief Interactive custom environment builder demo
 * 
 * This function provides an interactive interface for users to build custom environments
 * by adding obstacles (spheres, cuboids, capsules) and then running a planning demo.
 */
template<typename Robot>
bool runInteractiveCustomDemo(const std::string& robot_name)
{
    std::cout << "\n🎨 Custom Environment Builder" << std::endl;
    std::cout << "==============================" << std::endl;
    std::cout << "Build your custom environment by adding obstacles!" << std::endl;
    
    std::vector<ObstacleConfig> custom_obstacles;
    
    while (true) {
        std::cout << "\nCurrent environment has " << custom_obstacles.size() << " obstacles." << std::endl;
        std::cout << "What would you like to add?" << std::endl;
        std::cout << "1. Sphere" << std::endl;
        std::cout << "2. Cuboid (Box)" << std::endl;
        std::cout << "3. Capsule" << std::endl;
        std::cout << "4. Finish and run planning demo" << std::endl;
        std::cout << "5. Cancel and return to main menu" << std::endl;
        std::cout << "Enter choice (1-5): ";
        
        int choice;
        std::cin >> choice;
        
        if (choice == 4) {
            // Finish and run demo
            break;
        } else if (choice == 5) {
            // Cancel
            std::cout << "❌ Custom environment builder cancelled." << std::endl;
            return false;
        } else if (choice >= 1 && choice <= 3) {
            std::cout << "\nAdding obstacle..." << std::endl;
            std::cout << "Enter position (x y z): ";
            std::array<float, 3> position;
            std::cin >> position[0] >> position[1] >> position[2];
            
            if (choice == 1) {
                // Sphere
                std::cout << "Enter radius: ";
                float radius;
                std::cin >> radius;
                ObstacleConfig config("sphere", position, radius);
                config.name = "custom_sphere_" + std::to_string(custom_obstacles.size());
                custom_obstacles.push_back(config);
                std::cout << "✓ Added sphere at (" << position[0] << ", " << position[1] << ", " << position[2] << ") with radius " << radius << std::endl;
                
            } else if (choice == 2) {
                // Cuboid
                std::cout << "Enter half extents (x y z): ";
                std::array<float, 3> half_extents;
                std::cin >> half_extents[0] >> half_extents[1] >> half_extents[2];
                ObstacleConfig config("cuboid", position, half_extents);
                config.name = "custom_cuboid_" + std::to_string(custom_obstacles.size());
                custom_obstacles.push_back(config);
                std::cout << "✓ Added cuboid at (" << position[0] << ", " << position[1] << ", " << position[2] << ") with half extents (" << half_extents[0] << ", " << half_extents[1] << ", " << half_extents[2] << ")" << std::endl;
                
            } else if (choice == 3) {
                // Capsule
                std::cout << "Enter radius: ";
                float radius;
                std::cin >> radius;
                std::cout << "Enter length: ";
                float length;
                std::cin >> length;
                ObstacleConfig config("capsule", position, radius);
                config.length = length;
                config.orientation_euler_xyz = {0.0f, 0.0f, 1.0f}; // Default vertical orientation
                config.name = "custom_capsule_" + std::to_string(custom_obstacles.size());
                custom_obstacles.push_back(config);
                std::cout << "✓ Added capsule at (" << position[0] << ", " << position[1] << ", " << position[2] << ") with radius " << radius << " and length " << length << std::endl;
            }
        } else {
            std::cout << "Invalid choice. Please try again." << std::endl;
        }
    }
    
    // Create demo configuration with custom environment
    std::cout << "\nChoose planner for your custom environment:" << std::endl;
    std::cout << "1. BIT* (optimal)" << std::endl;
    std::cout << "2. RRT-Connect (fast)" << std::endl;
    std::cout << "3. PRM (multi-query)" << std::endl;
    std::cout << "Enter choice (1-3): ";
    
    int planner_choice;
    std::cin >> planner_choice;
    
    std::string planner_name = "BIT*";
    if (planner_choice == 2) planner_name = "RRT-Connect";
    else if (planner_choice == 3) planner_name = "PRM";
    
    // Create demo configuration
    DemoConfiguration config(robot_name, "custom", planner_name, 2.0, 1.0, false, true,
                           "Interactive Custom Environment: " + robot_name + " + " + planner_name);
    
    // Create environment factory with custom obstacles
    auto env_factory = createUnifiedEnvironmentFactory("custom", nullptr, &custom_obstacles, "Interactive Custom Environment");
    
    std::cout << "\n🚀 Running planning demo with your custom environment!" << std::endl;
    return executePlanning<Robot>(config, std::move(env_factory), "Interactive Custom Environment Demo");
}

/**
 * @brief Run all predefined demos
 */
inline void runAllDemos()
{
    std::cout << "\n🚀 VAMP + OMPL Integration Demo Suite" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "This demo showcases the integration of VAMP (Vector-Accelerated Motion Planning)" << std::endl;
    std::cout << "with OMPL using a clean, extensible architecture." << std::endl;
    
    auto demos = getPredefinedDemos();
    int success_count = 0;
    
    for (const auto& demo : demos) {
        bool success = dispatchByRobotType(demo.robot_name, [&demo]<typename Robot>() {
            return runSingleDemo<Robot>(demo);
        });
        
        if (success) success_count++;
    }
    
    std::cout << "\n🏁 Demo Suite Complete!" << std::endl;
    std::cout << "Success rate: " << success_count << "/" << demos.size() 
              << " (" << (100.0 * success_count / demos.size()) << "%)" << std::endl;
}

} // namespace vamp_ompl