#include "VampOMPLDemo.h"
#include <iostream>
#include <string>

using namespace vamp_ompl;

/**
 * @brief Simple example showing how to use the VAMP-OMPL integration
 */
void basicExample()
{
    std::cout << "\nBasic Usage Example" << std::endl;
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
 * @brief Interactive demo selector
 */
void interactiveDemo()
{
    std::cout << "\nInteractive Demo" << std::endl;
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
    std::cout << " VAMP + OMPL Integration Demo" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "A clean, extensible architecture for high-performance motion planning" << std::endl;
    
    try {
        if (argc > 1) {
            std::string mode = argv[1];
            
            if (mode == "basic") {
                basicExample();
            } else if (mode == "interactive") {
                interactiveDemo();
            } else {
                std::cout << "\nUsage: " << argv[0] << " [mode]" << std::endl;
                std::cout << "Modes:" << std::endl;
                std::cout << "  basic        - Basic usage example" << std::endl;
                std::cout << "  interactive  - Interactive demo selector" << std::endl;
                std::cout << "  (no args)    - Run all examples" << std::endl;
            }
        } else {
            // Run all examples by default
            basicExample();
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
} 