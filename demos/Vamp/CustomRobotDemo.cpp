/**
 * @file CustomRobotDemo.cpp
 * @brief Demonstration of custom robot registration and planning with VAMP
 * 
 * This demo shows how to:
 * 1. Include and register custom robots
 * 2. Create planning configurations for custom robots
 * 3. Execute motion planning using the registry system
 * 4. Visualize results with custom robots
 * 
 * Usage:
 *   ./demo_CustomRobot                           # Basic custom robot demo
 *   ./demo_CustomRobot --visualize               # With visualization
 */

#include "VampOMPLDemo.h"
#include "VampRobotRegistry.h"
#include "VampUtils.h"
#include "CustomRobotExample.h"  // This registers our custom robots
#include <iostream>
#include <string>

using namespace vamp_ompl;

/**
 * @brief Create example configuration for 2DOF planar arm
 */
PlanningConfiguration createPlanarArmExample()
{
    std::cout << " Creating 2DOF Planar Arm configuration..." << std::endl;
    
    PlanningConfiguration config;
    
    // Robot configuration
    config.robot_name = "planar_arm_2dof";
    config.description = "2DOF Planar Arm Navigation Demo";
    
    // Start configuration: arm pointing to the right
    config.start_config = {
        0.0f,      // shoulder: 0° (pointing right)
        0.0f       // elbow: 0° (straight)
    };
    
    // Goal configuration: arm folded up
    config.goal_config = {
        1.5708f,   // shoulder: 90° (pointing up)
        -1.5708f   // elbow: -90° (folded)
    };
    
    // Planning configuration
    config.planning.planner_name = "RRT-Connect";
    config.planning.planning_time = 2.0;
    config.planning.simplification_time = 1.0;
    config.planning.optimize_path = false;
    
    // Environment: Simple obstacle course
    std::cout << "Creating obstacle course for planar arm..." << std::endl;
    
    // Obstacle 1
    ObstacleConfig obstacle1;
    obstacle1.type = "sphere";
    obstacle1.name = "blocking_sphere";
    obstacle1.position = {0.2f, 0.0f, 0.0f};
    obstacle1.radius = 0.08f;
    config.obstacles.push_back(obstacle1);
    
    // Obstacle 2
    ObstacleConfig obstacle2;
    obstacle2.type = "cuboid";
    obstacle2.name = "wall_left";
    obstacle2.position = {0.15f, 0.2f, 0.0f};
    obstacle2.half_extents = {0.05f, 0.1f, 0.1f};
    config.obstacles.push_back(obstacle2);
    
    ObstacleConfig obstacle3;
    obstacle3.type = "cuboid";
    obstacle3.name = "wall_right";
    obstacle3.position = {0.15f, -0.2f, 0.0f};
    obstacle3.half_extents = {0.05f, 0.1f, 0.1f};
    config.obstacles.push_back(obstacle3);
    
    config.save_path = true;
    
    return config;
}

/**
 * @brief List all registered robots
 */
void listRegisteredRobots()
{
    auto& registry = RobotRegistry::getInstance();
    auto robots = registry.getRegisteredRobots();
    
    std::cout << "\n Registered Robots:" << std::endl;
    for (const auto& robot_name : robots) {
        try {
            auto metadata = registry.getRobotMetadata(robot_name);
            std::cout << "  - " << robot_name << " (" << metadata.dimension << " DOF): " 
                      << metadata.description << std::endl;
        } catch (const std::exception& e) {
            std::cout << "  - " << robot_name << " (metadata error)" << std::endl;
        }
    }
    std::cout << std::endl;
}

/**
 * @brief Show robot details
 */
void showRobotDetails(const std::string& robot_name)
{
    try {
        auto& registry = RobotRegistry::getInstance();
        auto metadata = registry.getRobotMetadata(robot_name);
        
        std::cout << "\n Robot Details: " << robot_name << std::endl;
        std::cout << "  Description: " << metadata.description << std::endl;
        std::cout << "  Dimensions: " << metadata.dimension << " DOF" << std::endl;
        std::cout << "  Collision spheres: " << metadata.n_spheres << std::endl;
        std::cout << "  End-effector: " << metadata.end_effector_frame << std::endl;
        std::cout << "  Radii range: [" << metadata.radii_range.first 
                  << ", " << metadata.radii_range.second << "]" << std::endl;
        
        std::cout << "  Joint names: ";
        for (size_t i = 0; i < metadata.joint_names.size(); ++i) {
            std::cout << metadata.joint_names[i];
            if (i < metadata.joint_names.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
        
        // Show joint limits
        auto limits = registry.getJointLimits(robot_name);
        std::cout << "  Joint limits:" << std::endl;
        for (size_t i = 0; i < limits.size(); ++i) {
            std::cout << "    " << metadata.joint_names[i] << ": [" 
                      << limits[i].first << ", " << limits[i].second << "]" << std::endl;
        }
        std::cout << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << " Error getting robot details: " << e.what() << std::endl;
    }
}

/**
 * @brief Print usage information
 */
void printUsage(const char* program_name)
{
    std::cout << "VAMP Custom Robot Demo\n" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << program_name << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --help                    Show this help message" << std::endl;
    std::cout << "  --list                    List all registered robots" << std::endl;
    std::cout << "  --robot <name>            Use specific robot (default: planar_arm_2dof)" << std::endl;
    std::cout << "  --info <name>             Show robot information" << std::endl;
    std::cout << "  --visualize               Enable visualization" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " --list" << std::endl;
    std::cout << "  " << program_name << " --info panda" << std::endl;
}

int main(int argc, char* argv[])
{
    try {
        std::cout << " VAMP Custom Robot Demo" << std::endl;
        std::cout << "=========================" << std::endl;
        
        // Parse command line arguments
        std::string selected_robot = "planar_arm_2dof";  // Default
        bool should_visualize = false;
        bool list_robots = false;
        std::string info_robot = "";
        
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            } else if (arg == "--list") {
                list_robots = true;
            } else if (arg == "--robot" && i + 1 < argc) {
                selected_robot = argv[++i];
            } else if (arg == "--info" && i + 1 < argc) {
                info_robot = argv[++i];
            } else if (arg == "--visualize") {
                should_visualize = true;
            } else {
                std::cerr << "Unknown argument: " << arg << std::endl;
                printUsage(argv[0]);
                return 1;
            }
        }
        
        // Handle list request
        if (list_robots) {
            listRegisteredRobots();
            return 0;
        }
        
        // Handle info request
        if (!info_robot.empty()) {
            showRobotDetails(info_robot);
            return 0;
        }
        
        // Show registered robots
        listRegisteredRobots();
        
        // Validate selected robot
        auto& registry = RobotRegistry::getInstance();
        if (!registry.isRobotRegistered(selected_robot)) {
            std::cerr << " Robot '" << selected_robot << "' not registered!" << std::endl;
            std::cerr << "Use --list to see available robots." << std::endl;
            return 1;
        }
        
        // Show robot details
        showRobotDetails(selected_robot);
        
        // Create planning configuration based on selected robot
        PlanningConfiguration config;
        
        if (selected_robot == "planar_arm_2dof") {
            config = createPlanarArmExample();
        } else {
            // For other robots, create a basic configuration
            std::cout << " Creating basic configuration for " << selected_robot << "..." << std::endl;
            
            auto metadata = registry.getRobotMetadata(selected_robot);
            auto limits = registry.getJointLimits(selected_robot);
            
            config.robot_name = selected_robot;
            config.description = "Custom robot demo: " + selected_robot;
            
            // Create start configuration (all joints at midrange)
            config.start_config.resize(metadata.dimension);
            config.goal_config.resize(metadata.dimension);
            
            for (size_t i = 0; i < metadata.dimension; ++i) {
                double mid = (limits[i].first + limits[i].second) * 0.5;
                double range = limits[i].second - limits[i].first;
                
                config.start_config[i] = static_cast<float>(mid - range * 0.2);  // 20% below mid
                config.goal_config[i] = static_cast<float>(mid + range * 0.2);   // 20% above mid
            }
            
            // Basic obstacle
            ObstacleConfig obstacle;
            obstacle.type = "sphere";
            obstacle.name = "test_obstacle";
            obstacle.position = {0.3f, 0.0f, 0.3f};
            obstacle.radius = 0.1f;
            config.obstacles.push_back(obstacle);
            
            config.planning.planner_name = "RRT-Connect";
            config.planning.planning_time = 2.0;
            config.save_path = true;
        }
        
        // Validate configuration
        if (!config.isValid()) {
            std::cerr << " Configuration validation failed: " << config.getValidationErrors() << std::endl;
            return 1;
        }
        
        std::cout << " Configuration validated successfully" << std::endl;
        
        // Execute planning using registry-based approach
        std::cout << "\n Starting motion planning with " << selected_robot << "..." << std::endl;
        
        MotionPlanningResult result = executeMotionPlanning(config);
        
        // Show results
        VampUtils::printResults(result);
        
        if (result.success()) {
            std::cout << "\n Planning succeeded!" << std::endl;
            
            // Visualization
            if (should_visualize) {
                std::cout << "\n Starting visualization..." << std::endl;
                if (!VampUtils::runVisualization(result, selected_robot + "_custom_demo")) {
                    std::cout << "  Visualization failed (planning still succeeded)" << std::endl;
                }
            } else {
                std::cout << "\n To visualize this result, run:" << std::endl;
                std::cout << "   " << argv[0] << " --robot " << selected_robot << " --visualize" << std::endl;
            }
        } else {
            std::cout << "\n Planning failed!" << std::endl;
            return 1;
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << " Fatal error: " << e.what() << std::endl;
        return 1;
    }
} 