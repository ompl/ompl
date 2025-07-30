#include "VampOMPLDemo.h"
#include "YamlConfig.h"
#include <iostream>
#include <string>
#include <filesystem>
#include <cstdlib>
#include <thread>
#include <chrono>

using namespace vamp_ompl;

/**
 * @brief Convert YAML config to DemoConfiguration
 */
DemoConfiguration yamlConfigToDemo(const VampYamlConfig& yaml_config)
{
    return DemoConfiguration(
        yaml_config.planning.robot.name,
        yaml_config.planning.environment.name,
        yaml_config.planning.planner.name,
        yaml_config.planning.planner.planning_time,
        yaml_config.planning.planner.simplification_time,
        yaml_config.planning.planner.optimize_path,
        yaml_config.planning.output.write_path,
        yaml_config.planning.output.description
    );
}

/**
 * @brief Find the most recent solution path file
 */
std::string findMostRecentSolutionPath()
{
    std::string most_recent_file = "";
    std::filesystem::file_time_type most_recent_time{};
    
    // Search current directory and common locations
    std::vector<std::string> search_dirs = {".", "demos/Vamp/", "../demos/Vamp/"};
    
    for (const auto& search_dir : search_dirs) {
        if (!std::filesystem::exists(search_dir)) continue;
        
        try {
            for (const auto& entry : std::filesystem::directory_iterator(search_dir)) {
                if (!entry.is_regular_file()) continue;
                
                std::string filename = entry.path().filename().string();
                if (filename.find("solution_path_") == 0 && 
                    filename.length() >= 4 && 
                    filename.substr(filename.length() - 4) == ".txt") {
                    auto file_time = entry.last_write_time();
                    if (most_recent_file.empty() || file_time > most_recent_time) {
                        most_recent_file = entry.path().string();
                        most_recent_time = file_time;
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not search directory " << search_dir << ": " << e.what() << std::endl;
        }
    }
    
    return most_recent_file;
}

/**
 * @brief Run visualization using Python script
 */
bool runVisualization(const VampYamlConfig& config, const std::string& solution_path_file = "")
{
    if (!config.visualization.enabled) {
        return true; // Not an error, just disabled
    }
    
    std::cout << "\n🎬 Starting Visualization..." << std::endl;
    
    // Determine which solution file to use
    std::string path_file = solution_path_file;
    if (path_file.empty()) {
        path_file = findMostRecentSolutionPath();
        if (path_file.empty()) {
            std::cout << "❌ No solution path file found for visualization" << std::endl;
            return false;
        }
        std::cout << "📁 Using solution file: " << path_file << std::endl;
    }
    
    if (!std::filesystem::exists(path_file)) {
        std::cout << "❌ Solution path file not found: " << path_file << std::endl;
        return false;
    }
    
    // Find the visualization script
    std::vector<std::string> script_locations = {
        "demos/Vamp/visualize_solution.py",
        "../demos/Vamp/visualize_solution.py", 
        "../../demos/Vamp/visualize_solution.py",
        "visualize_solution.py"
    };
    
    std::string script_path = "";
    for (const auto& location : script_locations) {
        if (std::filesystem::exists(location)) {
            script_path = location;
            break;
        }
    }
    
    if (script_path.empty()) {
        std::cout << "❌ Visualization script not found" << std::endl;
        return false;
    }
    
    // Build command
    std::ostringstream cmd;
    cmd << "python3 " << script_path;
    
    // Add robot and environment if specified
    std::string robot_name = config.visualization.robot.name.empty() ? 
                           config.planning.robot.name : config.visualization.robot.name;
    std::string env_name = config.visualization.environment.name.empty() ? 
                         config.planning.environment.name : config.visualization.environment.name;
    
    // Map environment names between planning and visualization
    if (env_name == "table") env_name = "table_scene";
    
    cmd << " --robot " << robot_name;
    cmd << " --environment " << env_name;
    cmd << " --duration " << config.visualization.animation.duration;
    
    if (config.visualization.animation.loop) {
        cmd << " --loop";
    }
    
    if (!config.visualization.animation.draw_trajectory) {
        cmd << " --no-trajectory";
    }
    
    if (!config.visualization.display.gui) {
        cmd << " --no-gui";
    }
    
    if (config.visualization.display.verbose) {
        cmd << " --verbose";
    }
    
    cmd << " \"" << path_file << "\"";
    
    std::cout << "🚀 Running: " << cmd.str() << std::endl;
    
    if (!config.visualization.auto_start) {
        std::cout << "\nPress Enter to start visualization...";
        std::cin.get();
    }
    
    // Run the command
    int result = std::system(cmd.str().c_str());
    
    if (result == 0) {
        std::cout << "✅ Visualization completed successfully" << std::endl;
        return true;
    } else {
        std::cout << "❌ Visualization failed with exit code: " << result << std::endl;
        return false;
    }
}

/**
 * @brief Find YAML configuration file
 */
std::string findYamlConfig(const std::string& yaml_file)
{
    // If the file exists as specified, use it directly
    if (std::filesystem::exists(yaml_file)) {
        return yaml_file;
    }
    
    // Search in common locations relative to build directory
    std::vector<std::string> search_paths = {
        yaml_file,  // As provided
        "../demos/Vamp/" + yaml_file,
        "../demos/Vamp/config/" + yaml_file,  // From build directory
    };

    for (const auto& path : search_paths) {
        std::cout << "   - " << path;
        if (std::filesystem::exists(path)) {
            std::cout << " ✓ FOUND" << std::endl;
            return path;
        } else {
            std::cout << " ✗ not found" << std::endl;
        }
    }
    
    return "";  // Not found
}

/**
 * @brief Run demo from YAML configuration
 */
bool runYamlDemo(const std::string& yaml_file)
{
    std::cout << "\n🎯 Running VAMP-OMPL Demo from YAML Configuration" << std::endl;
    std::cout << "=================================================" << std::endl;
    
    // Find the YAML configuration file
    std::string config_path = findYamlConfig(yaml_file);
    if (config_path.empty()) {
        std::cout << "❌ YAML configuration file not found: " << yaml_file << std::endl;
        std::cout << "💡 Searched in:" << std::endl;
        std::cout << "   - Current directory: " << yaml_file << std::endl;
        std::cout << "   - Config directory: config/" << yaml_file << std::endl;
        std::cout << "   - Source config: demos/Vamp/config/" << yaml_file << std::endl;
        return false;
    }
    
    std::cout << "📁 Using configuration: " << config_path << std::endl;
    
    // Load YAML configuration
    std::cout << "🔍 Debug: About to load YAML from: " << config_path << std::endl;
    VampYamlConfig yaml_config;
    std::cout << "🔍 Debug: Created VampYamlConfig object" << std::endl;
    
    bool load_result = yaml_config.loadFromFile(config_path);
    std::cout << "🔍 Debug: loadFromFile returned: " << (load_result ? "true" : "false") << std::endl;
    
    if (!load_result) {
        std::cout << "❌ Failed to load YAML configuration from: " << config_path << std::endl;
        return false;
    }
    
    // Print configuration
    yaml_config.print();
    
    // Convert to demo configuration
    auto demo_config = yamlConfigToDemo(yaml_config);
    
    bool planning_success = false;
    
    // Run planning based on robot type
    if (demo_config.robot_name == "panda") {
        planning_success = runSingleDemo<vamp::robots::Panda>(demo_config);
    } else if (demo_config.robot_name == "ur5") {
        planning_success = runSingleDemo<vamp::robots::UR5>(demo_config);
    } else if (demo_config.robot_name == "fetch") {
        planning_success = runSingleDemo<vamp::robots::Fetch>(demo_config);
    } else {
        std::cout << "❌ Unsupported robot: " << demo_config.robot_name << std::endl;
        return false;
    }
    
    if (!planning_success) {
        std::cout << "❌ Planning failed, skipping visualization" << std::endl;
        return false;
    }
    
    // Run visualization if enabled and planning succeeded
    if (yaml_config.visualization.enabled) {
        // Add a small delay to ensure file is written
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        runVisualization(yaml_config);
    }
    
    return planning_success;
}

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
        config.write_path = true;  // Enable path writing to demonstrate the feature
        
        // Step 6: Plan
        std::cout << "6. Planning..." << std::endl;
        auto result = planner->plan(config);
        
        // Step 7: Check results
        std::cout << "7. Results:" << std::endl;
        if (result.success) {
            std::cout << "   ✓ Planning succeeded!" << std::endl;
            std::cout << "   → Planning time: " << result.planning_time_us << " μs" << std::endl;
            std::cout << "   → Path length: " << result.path_length << " states" << std::endl;
            std::cout << "   → Path cost: " << result.final_cost << std::endl;
            
            if (config.write_path) {
                std::cout << "   ✓ Solution path written to file" << std::endl;
                std::cout << "\n💡 You can now visualize this path using:" << std::endl;
                std::cout << "   python3 demos/Vamp/visualize_solution.py --interactive" << std::endl;
            }
        } else {
            std::cout << "   ✗ Planning failed: " << result.error_message << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "❌ Error in basic example: " << e.what() << std::endl;
    }
}

void interactiveDemo()
{
    std::cout << "\nInteractive Demo" << std::endl;
    std::cout << "===================" << std::endl;
    std::cout << "Choose a demo to run:" << std::endl;
    std::cout << "1. Panda + Sphere Cage + BIT*" << std::endl;
    std::cout << "2. UR5 + Table Scene + RRT-Connect (with path saving)" << std::endl;
    std::cout << "3. Fetch + Empty Space + PRM" << std::endl;
    std::cout << "4. Custom Environment Builder 🎨" << std::endl;
    std::cout << "5. Predefined Custom Environments" << std::endl;
    std::cout << "Enter choice (1-5): ";
    
    int choice;
    std::cin >> choice;
    
    DemoConfiguration config;
    
    switch (choice) {
        case 1:
            config = DemoConfiguration("panda", "sphere_cage", "BIT*", 1.0, 0.5, false, false,
                                     "Interactive: Panda + Sphere Cage + BIT*");
            dispatchByRobotType(config.robot_name, [&config]<typename Robot>() {
                return runSingleDemo<Robot>(config);
            });
            break;
            
        case 2:
            config = DemoConfiguration("ur5", "table", "RRT-Connect", 1.0, 0.5, false, true,
                                     "Interactive: UR5 + Table Scene + RRT-Connect (with path saving)");
            dispatchByRobotType(config.robot_name, [&config]<typename Robot>() {
                return runSingleDemo<Robot>(config);
            });
            break;
            
        case 3:
            config = DemoConfiguration("fetch", "empty", "PRM", 1.0, 0.5, false, false,
                                     "Interactive: Fetch + Empty Space + PRM");
            dispatchByRobotType(config.robot_name, [&config]<typename Robot>() {
                return runSingleDemo<Robot>(config);
            });
            break;
            
        case 4:
            {
                std::cout << "\n🤖 Choose robot for custom environment:" << std::endl;
                std::cout << "1. Panda" << std::endl;
                std::cout << "2. UR5" << std::endl;
                std::cout << "3. Fetch" << std::endl;
                std::cout << "Enter choice (1-3): ";
                int robot_choice;
                std::cin >> robot_choice;
                
                std::string robot_name = "panda";  // default
                if (robot_choice == 2) robot_name = "ur5";
                else if (robot_choice == 3) robot_name = "fetch";
                else if (robot_choice != 1) {
                    std::cout << "Invalid choice. Using Panda as default." << std::endl;
                }
                
                dispatchByRobotType(robot_name, [&robot_name]<typename Robot>() {
                    return runInteractiveCustomDemo<Robot>(robot_name);
                });
            }
            break;
            
        case 5:
            {
                std::cout << "\nPredefined Custom Environments:" << std::endl;
                std::cout << "1. Mixed obstacles (spheres + cuboids + capsules)" << std::endl;
                std::cout << "2. Spheres only" << std::endl;
                std::cout << "3. Cuboids only" << std::endl;
                std::cout << "Enter choice (1-3): ";
                int env_choice;
                std::cin >> env_choice;
                
                std::string env_name = "custom_mixed";
                if (env_choice == 2) env_name = "custom_spheres";
                else if (env_choice == 3) env_name = "custom_cuboids";
                
                config = DemoConfiguration("panda", env_name, "BIT*", 2.0, 1.0, false, true,
                                         "Interactive: Panda + " + env_name + " + BIT* (with path saving)");
                dispatchByRobotType(config.robot_name, [&config]<typename Robot>() {
                    return runSingleDemo<Robot>(config);
                });
            }
            break;
            
        default:
            std::cout << "Invalid choice. Running default demo." << std::endl;
            config = DemoConfiguration("panda", "sphere_cage", "BIT*");
            dispatchByRobotType(config.robot_name, [&config]<typename Robot>() {
                return runSingleDemo<Robot>(config);
            });
            break;
    }
}

void printUsage(const char* program_name)
{
    std::cout << "\n🎯 VAMP + OMPL Integration Demo" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "A clean, extensible architecture for high-performance motion planning" << std::endl;
    std::cout << "with support for custom obstacle environments" << std::endl;
    std::cout << "\nUsage: " << program_name << " [mode] [config_file]" << std::endl;
    std::cout << "\nModes:" << std::endl;
    std::cout << "  basic              - Basic usage example" << std::endl;
    std::cout << "  interactive        - Interactive demo selector with custom environment builder" << std::endl;
    std::cout << "  yaml <config_file> - Run with YAML configuration (supports custom obstacles)" << std::endl;
    std::cout << "\nYAML Configuration:" << std::endl;
    std::cout << "  Use config files for configuration-driven planning and visualization" << std::endl;
    std::cout << "  Standard configs:" << std::endl;
    std::cout << "    " << program_name << " yaml config/vamp_config.yaml" << std::endl;
    std::cout << "    " << program_name << " yaml panda_demo.yaml" << std::endl;
    std::cout << "  Custom obstacle configs:" << std::endl;
    std::cout << "    " << program_name << " yaml config/custom_obstacles_demo.yaml" << std::endl;
    std::cout << "    " << program_name << " yaml config/simple_spheres_demo.yaml" << std::endl;
    std::cout << "\nCustom Environments:" << std::endl;
    std::cout << "  🎨 Interactive Mode: Build environments by specifying obstacle positions" << std::endl;
    std::cout << "  📝 YAML Mode: Define custom obstacles in YAML with sphere, cuboid, capsule types" << std::endl;
    std::cout << "  🔧 Predefined: Use built-in custom environment samples" << std::endl;
    std::cout << "\nSupported Obstacle Types:" << std::endl;
    std::cout << "  • Sphere: position + radius" << std::endl;
    std::cout << "  • Cuboid: position + half_extents + orientation" << std::endl;
    std::cout << "  • Capsule: position + orientation + radius + length" << std::endl;
    std::cout << "\nVisualization:" << std::endl;
    std::cout << "  When path writing is enabled, you can visualize results with:" << std::endl;
    std::cout << "  python3 demos/Vamp/visualize_solution.py --interactive" << std::endl;
}

int main(int argc, char **argv)
{
    try {
        if (argc > 1) {
            std::string mode = argv[1];
            
            if (mode == "basic") {
                basicExample();
            } else if (mode == "interactive") {
                interactiveDemo();
            } else if (mode == "yaml") {
                if (argc < 3) {
                    std::cout << "❌ YAML mode requires a configuration file" << std::endl;
                    std::cout << "Usage: " << argv[0] << " yaml <config_file>" << std::endl;
                    return 1;
                }
                
                std::string yaml_file = argv[2];
                
                if (!runYamlDemo(yaml_file)) {
                    return 1;
                }
            } else if (mode == "help" || mode == "--help" || mode == "-h") {
                printUsage(argv[0]);
            } else {
                std::cout << "❌ Unknown mode: " << mode << std::endl;
                printUsage(argv[0]);
                return 1;
            }
        } else {
            basicExample();
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
} 