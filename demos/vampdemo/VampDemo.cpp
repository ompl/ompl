#include "VampOMPLDemo.h"
#include <iostream>
#include <string>

using namespace vamp_ompl;

/**
 * @brief Simple example showing how to use the VAMP-OMPL integration
 */
void basicExample()
{
    std::cout << "\n📚 Basic Usage Example" << std::endl;
    std::cout << "=======================" << std::endl;
    std::cout << "This example shows the basic steps to use VAMP-OMPL integration:" << std::endl;
    
    try {
        // Step 1: Create robot configuration
        std::cout << "\n1. Creating robot configuration..." << std::endl;
        auto robot_config = createRobotConfig<vamp::robots::Panda>("panda", "");
        
        // Step 2: Create environment factory  
        std::cout << "2. Creating environment factory..." << std::endl;
        auto env_factory = createEnvironmentFactory("sphere_cage");
        
        // Step 3: Create planner
        std::cout << "3. Creating VAMP-OMPL planner..." << std::endl;
        auto planner = createVampOMPLPlanner<vamp::robots::Panda>(std::move(robot_config), std::move(env_factory));
        
        // Step 4: Initialize
        std::cout << "4. Initializing planner..." << std::endl;
        planner->initialize();
        
        // Step 5: Configure planning
        std::cout << "5. Configuring planning parameters..." << std::endl;
        PlanningConfig config;
        config.planner_name = "BIT*";
        config.planning_time = 1.0;
        config.simplification_time = 0.5;
        config.optimize_path = false;
        
        // Step 6: Plan
        std::cout << "6. Planning..." << std::endl;
        auto result = planner->plan(config);
        
        // Step 7: Check results
        std::cout << "7. Results:" << std::endl;
        if (result.success) {
            std::cout << "   ✓ Planning succeeded!" << std::endl;
            std::cout << "   ✓ Planning time: " << result.planning_time_us << " μs" << std::endl;
            std::cout << "   ✓ Path length: " << result.path_length << " states" << std::endl;
            std::cout << "   ✓ Path cost: " << result.final_cost << std::endl;
            } else {
            std::cout << "   ✗ Planning failed: " << result.error_message << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "   ✗ Error: " << e.what() << std::endl;
    }
}

/**
 * @brief Advanced example showing custom configurations
 */
void advancedExample()
{
    std::cout << "\n🔬 Advanced Usage Example" << std::endl;
    std::cout << "==========================" << std::endl;
    std::cout << "This example shows advanced features like custom configurations:" << std::endl;
    
    try {
        // Create custom environment with specific parameters
        std::cout << "\n1. Creating sphere cage environment..." << std::endl;
        auto env_factory = std::make_unique<SphereCageEnvironmentFactory>(0.15f); // sphere radius
        
        // Create UR5 robot configuration
        std::cout << "2. Creating UR5 robot configuration..." << std::endl;
        auto robot_config = createRobotConfig<vamp::robots::UR5>("ur5", "");

        // Create planner
        auto planner = createVampOMPLPlanner<vamp::robots::UR5>(std::move(robot_config), std::move(env_factory));
        
        planner->printConfiguration();
        planner->initialize();
        
        // Try multiple planners
        std::vector<std::string> planners = {"BIT*", "RRT-Connect", "PRM"};
        
        for (const auto& planner_name : planners) {
            std::cout << "\nTesting planner: " << planner_name << std::endl;
            
            PlanningConfig config(1.0, 0.5, false, planner_name);
            auto result = planner->plan(config);
            
            std::cout << "  " << (result.success ? "✓" : "✗") 
                      << " " << planner_name << ": ";
            if (result.success) {
                std::cout << result.planning_time_us << " μs, cost " << result.final_cost << std::endl;
            } else {
                std::cout << "Failed" << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cout << "✗ Error: " << e.what() << std::endl;
    }
}

/**
 * @brief Show how to extend the system with custom components
 */
void extensibilityExample()
{
    std::cout << "\n🔧 Extensibility Example" << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "This shows how easy it is to add new components:" << std::endl;
    
    // Show that we have different working environments
    auto empty_factory = std::make_unique<EmptyEnvironmentFactory>();
    auto sphere_factory = std::make_unique<SphereCageEnvironmentFactory>();
    auto table_factory = std::make_unique<TableSceneEnvironmentFactory>();
    
    std::cout << "\nAvailable environments:" << std::endl;
    std::cout << "  - " << empty_factory->getEnvironmentName() << ": " << empty_factory->getDescription() << std::endl;
    std::cout << "  - " << sphere_factory->getEnvironmentName() << ": " << sphere_factory->getDescription() << std::endl;
    std::cout << "  - " << table_factory->getEnvironmentName() << ": " << table_factory->getDescription() << std::endl;
    
    // This demonstrates how easy it is to create new combinations
    // Users can easily add:
    // - New robot configurations by inheriting from RobotConfig<Robot>
    // - New environments by inheriting from EnvironmentFactory
    // - Custom planning workflows by using the building blocks
    
    std::cout << "\n📝 To add new components:" << std::endl;
    std::cout << "  1. New robots: Inherit from RobotConfig<YourRobot>" << std::endl;
    std::cout << "  2. New environments: Inherit from EnvironmentFactory" << std::endl;
    std::cout << "  3. New workflows: Compose existing components" << std::endl;
    std::cout << "  4. Everything is modular and testable!" << std::endl;
}

/**
 * @brief Interactive demo selector
 */
void interactiveDemo()
{
    std::cout << "\n🎮 Interactive Demo" << std::endl;
    std::cout << "===================" << std::endl;
    std::cout << "Choose a demo to run:" << std::endl;
    std::cout << "1. Panda + Sphere Cage + BIT*" << std::endl;
    std::cout << "2. UR5 + Table Scene + RRT-Connect" << std::endl;
    std::cout << "3. Fetch + Empty Space + PRM" << std::endl;
    std::cout << "4. Custom demo configuration" << std::endl;
    std::cout << "Enter choice (1-4): ";
    
    int choice;
    std::cin >> choice;
    
    DemoConfiguration config;
    
    switch (choice) {
        case 1:
            config = DemoConfiguration("panda", "sphere_cage", "BIT*", 1.0, 0.5, false,
                                     "Interactive: Panda + Sphere Cage + BIT*");
            runSingleDemo<vamp::robots::Panda>(config);
            break;
            
        case 2:
            config = DemoConfiguration("ur5", "table", "RRT-Connect", 1.0, 0.5, false,
                                     "Interactive: UR5 + Table Scene + RRT-Connect");
            runSingleDemo<vamp::robots::UR5>(config);
            break;
            
        case 3:
            config = DemoConfiguration("fetch", "empty", "PRM", 1.0, 0.5, false,
                                     "Interactive: Fetch + Empty Space + PRM");
            runSingleDemo<vamp::robots::Fetch>(config);
                    break;
            
                case 4:
            std::cout << "Custom demo: Using UR5 in sphere cage with BIT*" << std::endl;
            config = DemoConfiguration("ur5", "sphere_cage", "BIT*", 2.0, 1.0, false,
                                     "Interactive: Custom UR5 Sphere Cage Challenge");
            runSingleDemo<vamp::robots::UR5>(config);
            break;
            
        default:
            std::cout << "Invalid choice. Running default demo." << std::endl;
            config = DemoConfiguration("panda", "sphere_cage", "BIT*");
            runSingleDemo<vamp::robots::Panda>(config);
                    break;
    }
}

int main(int argc, char **argv)
{
    std::cout << "🤖 VAMP + OMPL Integration Demo" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "A clean, extensible architecture for high-performance motion planning" << std::endl;
    
    try {
        if (argc > 1) {
            std::string mode = argv[1];
            
            if (mode == "basic") {
                basicExample();
            } else if (mode == "advanced") {
                advancedExample();
            } else if (mode == "extensibility") {
                extensibilityExample();
            } else if (mode == "interactive") {
                interactiveDemo();
            } else if (mode == "all") {
                runAllDemos();
            } else {
                std::cout << "\nUsage: " << argv[0] << " [mode]" << std::endl;
                std::cout << "Modes:" << std::endl;
                std::cout << "  basic        - Basic usage example" << std::endl;
                std::cout << "  advanced     - Advanced features example" << std::endl;
                std::cout << "  extensibility- Show how to extend the system" << std::endl;
                std::cout << "  interactive  - Interactive demo selector" << std::endl;
                std::cout << "  all          - Run all predefined demos" << std::endl;
                std::cout << "  (no args)    - Run all examples" << std::endl;
            }
        } else {
            // Run all examples by default
            basicExample();
            advancedExample();
            extensibilityExample();
            runAllDemos();
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
} 