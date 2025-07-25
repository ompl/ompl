#pragma once

// Include all VAMP-OMPL integration components
#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include "OMPLPlanningContext.h"
#include "RobotConfigurations.h"
#include "EnvironmentFactories.h"
#include "VampOMPLPlanner.h"

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
    std::string description;         ///< Description of this demo
    
    DemoConfiguration(const std::string& robot = "panda",
                     const std::string& env = "sphere_cage", 
                     const std::string& planner = "BIT*",
                     double plan_time = 1.0,
                     double simp_time = 0.5,
                     bool optimize = false,
                     const std::string& desc = "VAMP-OMPL Demo")
        : robot_name(robot), environment_name(env), planner_name(planner),
          planning_time(plan_time), simplification_time(simp_time),
          optimize_path(optimize), description(desc)
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
    } else {
        throw std::invalid_argument("Unknown environment: " + env_name + 
                                  ". Available: empty, sphere_cage, table");
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
                         demo_config.planner_name);
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
        std::cout << "Planning time: " << result.planning_time_us << " μs" << std::endl;
        std::cout << "Simplification time: " << result.simplification_time_us << " μs" << std::endl;
        std::cout << "Initial path cost: " << result.initial_cost << std::endl;
        std::cout << "Final path cost: " << result.final_cost << std::endl;
        std::cout << "Path length: " << result.path_length << " states" << std::endl;
        
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
 */
template<typename Robot>
bool runSingleDemo(const DemoConfiguration& demo_config)
{
    try {
        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "Starting Demo: " << demo_config.description << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        
        // Create robot and environment configurations
        auto robot_config = createRobotConfig<Robot>(demo_config.robot_name, demo_config.environment_name);
        auto env_factory = createEnvironmentFactory(demo_config.environment_name);
        
        // Create planner
        auto planner = createVampOMPLPlanner(std::move(robot_config), std::move(env_factory));
        
        // Print configuration
        planner->printConfiguration();
        
        // Initialize
        planner->initialize();
        
        // Create planning configuration
        auto planning_config = createPlanningConfig(demo_config);
        
        // Plan
        std::cout << "\nStarting planning..." << std::endl;
        auto result = planner->plan(planning_config);
        
        // Print results
        printPlanningResults(demo_config, result);
        
        return result.success;
        
    } catch (const std::exception& e) {
        std::cout << "✗ Demo failed: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Get predefined demo configurations
 */
inline std::vector<DemoConfiguration> getPredefinedDemos()
{
    return {
        DemoConfiguration("panda", "sphere_cage", "RRT-Connect", 1.0, 0.5, false, 
                         "Panda in Sphere Cage with RRT-Connect"),
        DemoConfiguration("ur5", "sphere_cage", "PRM", 1.0, 0.5, false,
                         "UR5 in Sphere Cage with PRM"),
        DemoConfiguration("fetch", "empty", "RRT-Connect", 1.0, 0.5, false,
                         "Fetch in Empty Environment with RRT-Connect"),
        DemoConfiguration("panda", "table", "RRT-Connect", 1.0, 0.5, false,
                         "Panda in Table Scene with RRT-Connect")
    };
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
        bool success = false;
        
        if (demo.robot_name == "panda") {
            success = runSingleDemo<vamp::robots::Panda>(demo);
        } else if (demo.robot_name == "ur5") {
            success = runSingleDemo<vamp::robots::UR5>(demo);
        } else if (demo.robot_name == "fetch") {
            success = runSingleDemo<vamp::robots::Fetch>(demo);
        }
        
        if (success) success_count++;
    }
    
    std::cout << "\n🏁 Demo Suite Complete!" << std::endl;
    std::cout << "Success rate: " << success_count << "/" << demos.size() 
              << " (" << (100.0 * success_count / demos.size()) << "%)" << std::endl;
}

} // namespace vamp_ompl