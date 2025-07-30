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
 * @brief Interactive environment builder helper
 */
std::unique_ptr<CustomEnvironmentFactory> buildInteractiveEnvironment()
{
    std::cout << "\n🎨 Custom Environment Builder" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Let's build a custom environment with obstacles!" << std::endl;
    
    auto factory = std::make_unique<CustomEnvironmentFactory>(
        std::vector<ObstacleConfig>{}, 
        "Interactive Custom Environment", 
        "Environment built interactively"
    );
    
    std::cout << "\nAvailable obstacle types:" << std::endl;
    std::cout << "1. Sphere (position + radius)" << std::endl;
    std::cout << "2. Cuboid (position + half_extents + orientation)" << std::endl;
    std::cout << "3. Capsule (position + orientation + radius + length)" << std::endl;
    std::cout << "4. Use predefined sample environment" << std::endl;
    std::cout << "5. Finish and start planning" << std::endl;
    
    while (true) {
        std::cout << "\nEnter choice (1-5): ";
        int choice;
        std::cin >> choice;
        
        if (choice == 5) break;
        
        if (choice == 4) {
            std::cout << "\nPredefined samples:" << std::endl;
            std::cout << "1. Mixed obstacles (spheres + cuboids + capsules)" << std::endl;
            std::cout << "2. Spheres only" << std::endl;
            std::cout << "3. Cuboids only" << std::endl;
            std::cout << "Choose sample (1-3): ";
            int sample_choice;
            std::cin >> sample_choice;
            
            std::string sample_name = "mixed";
            if (sample_choice == 2) sample_name = "spheres_only";
            else if (sample_choice == 3) sample_name = "cuboids_only";
            
            factory = CustomEnvironmentFactory::createSampleEnvironment(sample_name);
            std::cout << "✓ Loaded predefined environment: " << sample_name << std::endl;
            factory->printConfiguration();
            continue;
        }
        
        if (choice == 1) {
            // Add sphere
            std::cout << "\n📍 Adding Sphere" << std::endl;
            std::cout << "Enter position (x y z): ";
            float x, y, z;
            std::cin >> x >> y >> z;
            std::cout << "Enter radius: ";
            float radius;
            std::cin >> radius;
            
            factory->addSphere({x, y, z}, radius);
            std::cout << "✓ Added sphere at (" << x << ", " << y << ", " << z << ") with radius " << radius << std::endl;
            
        } else if (choice == 2) {
            // Add cuboid
            std::cout << "\n📦 Adding Cuboid" << std::endl;
            std::cout << "Enter position (x y z): ";
            float x, y, z;
            std::cin >> x >> y >> z;
            std::cout << "Enter half-extents (half_x half_y half_z): ";
            float hx, hy, hz;
            std::cin >> hx >> hy >> hz;
            std::cout << "Enter orientation angles in radians (roll pitch yaw) [or 0 0 0 for axis-aligned]: ";
            float roll, pitch, yaw;
            std::cin >> roll >> pitch >> yaw;
            
            factory->addCuboid({x, y, z}, {hx, hy, hz}, {roll, pitch, yaw});
            std::cout << "✓ Added cuboid at (" << x << ", " << y << ", " << z << ")" << std::endl;
            
        } else if (choice == 3) {
            // Add capsule
            std::cout << "\n🔗 Adding Capsule" << std::endl;
            std::cout << "Enter position (x y z): ";
            float x, y, z;
            std::cin >> x >> y >> z;
            std::cout << "Enter orientation angles in radians (roll pitch yaw): ";
            float roll, pitch, yaw;
            std::cin >> roll >> pitch >> yaw;
            std::cout << "Enter radius: ";
            float radius;
            std::cin >> radius;
            std::cout << "Enter length: ";
            float length;
            std::cin >> length;
            
            factory->addCapsule({x, y, z}, {roll, pitch, yaw}, radius, length);
            std::cout << "✓ Added capsule at (" << x << ", " << y << ", " << z << ")" << std::endl;
            
        } else {
            std::cout << "Invalid choice. Please try again." << std::endl;
            continue;
        }
        
        std::cout << "\nCurrent obstacles: " << factory->getObstacles().size() << std::endl;
    }
    
    if (factory->getObstacles().empty()) {
        std::cout << "\n⚠️ No obstacles added. Using empty environment." << std::endl;
        factory = std::make_unique<CustomEnvironmentFactory>(
            std::vector<ObstacleConfig>{}, 
            "Empty Custom Environment", 
            "Empty environment created interactively"
        );
    }
    
    // Print final configuration
    std::cout << "\n🎯 Final Environment Configuration:" << std::endl;
    factory->printConfiguration();
    
    return factory;
}

/**
 * @brief Interactive planning configuration
 */
DemoConfiguration configureInteractivePlanning(const std::string& robot_name)
{
    std::cout << "\n⚙️ Planning Configuration" << std::endl;
    std::cout << "Enter planning time (seconds) [default: 2.0]: ";
    double planning_time = 2.0;
    std::cin >> planning_time;
    
    std::cout << "Write solution path to file? (1=yes, 0=no) [default: 1]: ";
    int write_path = 1;
    std::cin >> write_path;
    
    return DemoConfiguration(robot_name, "custom", "BIT*", planning_time, 0.5, false, (write_path == 1),
                           "Interactive Custom Environment Demo");
}

/**
 * @brief Interactive custom environment demo
 */
template<typename Robot>
bool runInteractiveCustomDemo(const std::string& robot_name)
{
    // Build environment interactively
    auto factory = buildInteractiveEnvironment();
    
    // Store obstacle information for visualization before moving the factory
    bool has_obstacles = !factory->getObstacles().empty();
    std::string obstacles_data = has_obstacles ? factory->serializeObstacles() : "";
    
    // Configure planning interactively
    auto demo_config = configureInteractivePlanning(robot_name);
    
    // Execute planning using unified function
    bool success = executePlanning<Robot>(demo_config, std::move(factory), "Interactive Custom Environment Demo");
    
    // Run visualization if planning succeeded and path was written
    if (success && demo_config.write_path && has_obstacles) {
        std::cout << "\n🎬 Starting Visualization..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small delay
        
        // Create a basic visualization config for custom environment
        VampYamlConfig viz_config;
        viz_config.visualization.enabled = true;
        viz_config.visualization.auto_start = true;
        viz_config.visualization.animation.duration = 8.0;
        viz_config.visualization.animation.loop = false;
        viz_config.visualization.animation.draw_trajectory = true;
        viz_config.visualization.display.gui = true;
        viz_config.visualization.display.verbose = false;
        viz_config.planning.robot.name = robot_name;
        viz_config.planning.environment.name = "custom";
        
        runVisualization(viz_config, "", obstacles_data);
    }
    
    return success;
}

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