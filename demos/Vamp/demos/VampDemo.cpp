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
#include "VampOMPLDemo.h"
#include "VampBenchmarkManager.h"
#include "VampUtils.h"
#include "VampRobotRegistry.h"
#include "CustomRobotExample.h"  // This registers our custom robots
#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <vector>
#include <map>

using namespace vamp_ompl;
using namespace vamp_ompl::benchmarking;

// Forward declarations for helper functions
bool run_single_planning_example(const PlanningConfiguration& config, bool visualize, const std::string& yaml_file = "");
std::string run_benchmark_example(const BenchmarkConfiguration& config);
bool run_quick_robot_benchmark(const std::string& robot_name);
void print_usage(const char* program_name);
void list_registered_robots();
PlanningConfiguration create_basic_example();
PlanningConfiguration create_planar_arm_example();

int main(int argc, char* argv[]) {
    std::cout << " VAMP Motion Planning & Benchmarking Demo" << std::endl;

    // Initialize the robot registry once
    RobotRegistry::getInstance();

    try {
        // Parse command line arguments
        if (argc == 1) {
            // Default: run basic programmatic planning example
            PlanningConfiguration basic_config = create_basic_example();
            return run_single_planning_example(basic_config, false) ? 0 : 1;
        } else if (argc >= 2) {
            std::string arg1 = argv[1];

            if (arg1 == "--help" || arg1 == "-h") {
                print_usage(argv[0]);
                return 0;
            } else if (arg1 == "--list-robots") {
                list_registered_robots();
                return 0;
            } else if (arg1 == "--robot" && argc == 3) {
                return run_quick_robot_benchmark(argv[2]) ? 0 : 1;
            } else if (arg1 == "--planar-arm" || arg1 == "planar_arm" || arg1 == "planar-arm") {
                // Run planar arm example
                bool benchmark_mode = false;
                bool visualize_mode = false;
                
                // Check for --benchmark or --visualize flags
                for (int i = 2; i < argc; ++i) {
                    std::string current_arg = argv[i];
                    if (current_arg == "--benchmark") {
                        benchmark_mode = true;
                    } else if (current_arg == "--visualize") {
                        visualize_mode = true;
                    }
                }
                
                if (benchmark_mode) {
                    // Run benchmarking for planar arm
                    return run_quick_robot_benchmark("planar_arm_2dof") ? 0 : 1;
                } else {
                    // Run single planning example
                    PlanningConfiguration planar_config = create_planar_arm_example();
                    return run_single_planning_example(planar_config, visualize_mode) ? 0 : 1;
                }
            } else if (arg1.size() >= 5 && arg1.substr(arg1.size() - 5) == ".yaml") {
                // YAML file provided
                PlanningConfiguration planning_config;
                BenchmarkConfiguration benchmark_config;
                bool is_benchmark_mode = false;
                bool visualize_mode = false;

                // Check for --benchmark or --visualize flags
                for (int i = 2; i < argc; ++i) {
                    std::string current_arg = argv[i];
                    if (current_arg == "--benchmark") {
                        is_benchmark_mode = true;
                    } else if (current_arg == "--visualize") {
                        visualize_mode = true;
                    }
                }

                if (is_benchmark_mode) {
                    if (!loadBenchmarkConfiguration(arg1, benchmark_config)) {
                        std::cerr << " Failed to load benchmark configuration from: " << arg1 << std::endl;
                        return 1;
                    }
                    if (!benchmark_config.isValid()) {
                        std::cerr << " Invalid benchmark configuration: " << benchmark_config.getValidationErrors() << std::endl;
                        return 1;
                    }
                    std::string logFilePath = run_benchmark_example(benchmark_config);
                    std::cout << " Unified demo benchmark completed. Log: " << logFilePath << std::endl;
                    return 0;
                } else {
                    if (!VampUtils::loadYamlConfig(arg1, planning_config)) {
                        std::cerr << " Failed to load planning configuration from: " << arg1 << std::endl;
                        return 1;
                    }
                    if (!planning_config.isValid()) {
                        std::cerr << " Invalid planning configuration: " << planning_config.getValidationErrors() << std::endl;
                        return 1;
                    }
                    return run_single_planning_example(planning_config, visualize_mode, arg1) ? 0 : 1;
                }
            } else {
                // Check if the argument might be a robot name for quick benchmark
                auto& registry = RobotRegistry::getInstance();
                if (registry.isRobotRegistered(arg1) && registry.isBenchmarkingAvailable(arg1)) {
                    std::cout << " Detected robot name '" << arg1 << "', running quick benchmark..." << std::endl;
                    return run_quick_robot_benchmark(arg1) ? 0 : 1;
                } else {
                    std::cerr << " Unknown argument or invalid usage: '" << arg1 << "'" << std::endl;
                    if (registry.isRobotRegistered(arg1)) {
                        std::cerr << " Note: Robot '" << arg1 << "' is registered but does not support benchmarking." << std::endl;
                    }
                    std::cerr << " Use --list-robots to see available robots." << std::endl;
                    print_usage(argv[0]);
                    return 1;
                }
            }
        } else {
            std::cerr << " Too many arguments" << std::endl;
            print_usage(argv[0]);
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << " Fatal error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

// Helper function implementations
bool run_single_planning_example(const PlanningConfiguration& config, bool visualize, const std::string& yaml_file) {
    std::cout << "\n Running single planning example for robot: " << config.robot_name << std::endl;
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto motion_planning_result = executeMotionPlanning(config);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        std::cout << "  Planning completed in " << duration.count() << " ms" << std::endl;
        
        if (motion_planning_result.success()) {
            std::cout << " Planning successful!" << std::endl;
            auto& planning_result = motion_planning_result.planning_result;
            std::cout << " Path length: " << planning_result.path_length << std::endl;
            std::cout << " Solution found with " << planning_result.path_length << " waypoints" << std::endl;
            
            // Show solution file path if path was written
            if (config.save_path && !motion_planning_result.solution_file_path.empty()) {
                std::cout << " Solution written to: " << motion_planning_result.solution_file_path << std::endl;
                
                if (visualize) {
                    std::cout << "\n Starting visualization..." << std::endl;
                    if (!VampUtils::runVisualization(motion_planning_result, config.robot_name + "_demo", yaml_file)) {
                        std::cout << "  Visualization failed (planning still succeeded)" << std::endl;
                        std::cout << " To visualize manually: python3 visualize_solution.py " << motion_planning_result.solution_file_path;
                        if (!yaml_file.empty()) {
                            std::cout << " --yaml-config " << yaml_file;
                        }
                        std::cout << std::endl;
                    }
                }
            }
            
            return true;
        } else {
            std::cout << " Planning failed: " << motion_planning_result.planning_result.error_message << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << " Planning failed: " << e.what() << std::endl;
        return false;
    }
}

std::string run_benchmark_example(const BenchmarkConfiguration& config) {
    std::cout << "\n Running benchmark example for experiment: " << config.experiment_name << std::endl;
    
    try {
        if (config.base_config.robot_name == "panda") {
            auto robotConfig = std::make_unique<RobotConfiguration<vamp::robots::Panda>>(
                config.base_config.robot_name,
                config.base_config.start_config,
                config.base_config.goal_config);

            auto envFactory = VampUtils::createEnvironmentFactory(config.base_config.obstacles, config.base_config.robot_name);

            auto benchmarkManager = createBenchmarkManager<vamp::robots::Panda>(
                std::move(robotConfig), std::move(envFactory));

            benchmarkManager->initialize();
            std::string logFilePath = benchmarkManager->executeBenchmark(config);
            
            std::cout << " Benchmark completed!" << std::endl;
            std::cout << " OMPL log file: " << logFilePath << std::endl;
            std::cout << "\n Generate database: ompl_benchmark_statistics.py " << logFilePath << " -d benchmark.db" << std::endl;
            std::cout << " Visualize at: http://plannerarena.org (upload benchmark.db)" << std::endl;
            
            return logFilePath;
        } else if (config.base_config.robot_name == "planar_arm_2dof") {
            auto robotConfig = std::make_unique<RobotConfiguration<vamp::robots::PlanarArm2DOF>>(
                config.base_config.robot_name,
                config.base_config.start_config,
                config.base_config.goal_config);

            auto envFactory = VampUtils::createEnvironmentFactory(config.base_config.obstacles, config.base_config.robot_name);

            auto benchmarkManager = createBenchmarkManager<vamp::robots::PlanarArm2DOF>(
                std::move(robotConfig), std::move(envFactory));

            benchmarkManager->initialize();
            std::string logFilePath = benchmarkManager->executeBenchmark(config);
            
            std::cout << " Benchmark completed!" << std::endl;
            std::cout << " OMPL log file: " << logFilePath << std::endl;
            std::cout << "\n Generate database: ompl_benchmark_statistics.py " << logFilePath << " -d benchmark.db" << std::endl;
            std::cout << " Visualize at: http://plannerarena.org (upload benchmark.db)" << std::endl;
            
            return logFilePath;
        } else {
            throw VampConfigurationError("Robot type '" + config.base_config.robot_name +
                                       "' not supported in unified benchmark. Use programmatic approach for custom robots.");
        }
    } catch (const std::exception& e) {
        std::cerr << " Benchmark failed: " << e.what() << std::endl;
        return "";
    }
}

bool run_quick_robot_benchmark(const std::string& robot_name) {
    std::cout << "\n Running quick benchmark for robot: " << robot_name << std::endl;
    
    auto& registry = RobotRegistry::getInstance();
    
    // Check if robot is registered and supports benchmarking
    if (!registry.isRobotRegistered(robot_name)) {
        std::cerr << " Robot '" << robot_name << "' is not registered" << std::endl;
        return false;
    }
    
    if (!registry.isBenchmarkingAvailable(robot_name)) {
        std::cerr << " Robot '" << robot_name << "' does not support benchmarking" << std::endl;
        return false;
    }
    
    try {
        // Get robot metadata for configuration
        auto metadata = registry.getRobotMetadata(robot_name);
        
        // Create robot-specific start and goal configurations
        std::vector<float> start_config, goal_config;
        
        if (robot_name == "panda") {
            start_config = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
            goal_config = {2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785};
        } else if (robot_name == "ur5") {
            start_config = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
            goal_config = {1.57, -0.785, 0.0, -1.57, 0.0, 0.0};
        } else if (robot_name == "fetch") {
            start_config = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0};
            goal_config = {0.3, -0.785, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0};
        } else if (robot_name == "planar_arm_2dof") {
            start_config = {0.7854, 0.0};    // 45° shoulder, 0° elbow 
            goal_config = {-0.7854, 0.0};   // -45° shoulder, 0° elbow
        } else {
            // Generic configuration for unknown robots
            start_config.resize(metadata.dimension, 0.0f);
            goal_config.resize(metadata.dimension, 0.5f);
        }
        
        // Create environment factory with simple obstacle
        std::vector<ObstacleConfig> obstacles = {
            ObstacleConfig("sphere", {0.5f, 0.0f, 0.5f}, 0.15f)
        };
        auto env_factory = VampUtils::createEnvironmentFactory(obstacles, robot_name);
        
        // Create benchmark manager through registry
        auto benchmark_manager = registry.createBenchmarkManager(
            robot_name, start_config, goal_config, std::move(env_factory));
        
        // Execute benchmark
        std::vector<std::string> planner_names = {"RRT-Connect", "BIT*"};
        auto results = registry.executeBenchmark(
            robot_name, benchmark_manager, 
            "Quick_" + robot_name + "_Benchmark",
            planner_names, 10, 3.0);
        
        // Print results
        std::cout << " Benchmark completed successfully!" << std::endl;
        std::cout << " Log file: " << results["log_file"] << std::endl;
        std::cout << " Status: " << results["status"] << std::endl;
        if (results.count("file_size")) {
            std::cout << " File size: " << results["file_size"] << " bytes" << std::endl;
        }
        
        return results["status"] == "completed";
        
    } catch (const std::exception& e) {
        std::cerr << " Quick benchmark failed: " << e.what() << std::endl;
        return false;
    }
}

void print_usage(const char* program_name) {
    std::cout << "\n VAMP Unified Motion Planning & Benchmarking Demo\n";
    std::cout << "Usage: " << program_name << " [options] [config.yaml]\n\n";
    std::cout << "Modes:\n";
    std::cout << "  (no args)                 - Run basic programmatic planning example\n";
    std::cout << "  <config.yaml>             - Run single planning from YAML config\n";
    std::cout << "  <config.yaml> --visualize - Run single planning with visualization\n";
    std::cout << "  <config.yaml> --benchmark - Run benchmark from YAML config\n";
    std::cout << "  --robot <robot_name>      - Run quick benchmark for specific robot\n";
    std::cout << "  <robot_name>              - Run quick benchmark for robot (shorthand)\n";
    std::cout << "  --planar-arm | planar_arm - Run 2DOF planar arm planning example\n";
    std::cout << "  --list-robots             - Show available robots\n";
    std::cout << "  --help                    - Show this help message\n\n";
    std::cout << "Examples:\n";
    std::cout << "  " << program_name << "                           # Basic planning\n";
    std::cout << "  " << program_name << " panda_demo.yaml          # YAML planning\n";
    std::cout << "  " << program_name << " panda_demo.yaml --benchmark  # Full benchmark\n";
    std::cout << "  " << program_name << " --robot panda            # Quick benchmark\n";
    std::cout << "  " << program_name << " panda                    # Quick benchmark (shorthand)\n";
    std::cout << "  " << program_name << " ur5                      # Quick benchmark for UR5\n";
    std::cout << "  " << program_name << " fetch                    # Quick benchmark for Fetch\n";
    std::cout << "  " << program_name << " --planar-arm             # 2DOF planar arm demo\n";
    std::cout << "  " << program_name << " planar_arm               # 2DOF planar arm demo (shorthand)\n";
    std::cout << "  " << program_name << " planar_arm --visualize   # With visualization\n";
    std::cout << "  " << program_name << " planar_arm --benchmark   # Benchmark planar arm\n\n";
}

void list_registered_robots() {
    std::cout << "\n Available Robots\n";
    std::cout << "==================\n";
    auto& registry = RobotRegistry::getInstance();
    auto robots = registry.getRegisteredRobots();
    for (const auto& robot : robots) {
        auto metadata = registry.getRobotMetadata(robot);
        std::cout << robot << " (" << metadata.dimension << " DOF): "
                  << metadata.description << "\n";
    }
    std::cout << "\n";
}

PlanningConfiguration create_basic_example() {
    PlanningConfiguration config;
    
    // Basic Panda configuration
    config.robot_name = "panda";
    config.planning.planner_name = "RRT-Connect";
    config.planning.planning_time = 5.0;
    
    // Add planner-specific parameters to match YAML configuration
    config.planning.planner_parameters["range"] = "0.3";
    config.planning.planner_parameters["intermediate_states"] = "false";
    
    // Joint configurations (7-DOF Panda)
    config.start_config = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    config.goal_config = {2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785};
    
    // Sphere cage environment with lower and upper rings
    std::vector<ObstacleConfig> obstacles;
    
    // Lower ring of spheres (z = 0.25) - creates workspace boundary
    std::vector<std::array<float, 3>> lowerRingObstaclePositions = {
        {0.55f, 0.0f, 0.25f}, {0.35f, 0.35f, 0.25f}, {0.0f, 0.55f, 0.25f},
        {-0.55f, 0.0f, 0.25f}, {-0.35f, -0.35f, 0.25f}, {0.0f, -0.55f, 0.25f},
        {0.35f, -0.35f, 0.25f}
    };

    // Upper ring of spheres (z = 0.8) - creates overhead constraints
    std::vector<std::array<float, 3>> upperRingObstaclePositions = {
        {0.35f, 0.35f, 0.8f}, {0.0f, 0.55f, 0.8f}, {-0.35f, 0.35f, 0.8f},
        {-0.55f, 0.0f, 0.8f}, {-0.35f, -0.35f, 0.8f}, {0.0f, -0.55f, 0.8f},
        {0.35f, -0.35f, 0.8f}
    };
    
    // Add lower ring obstacles
    for (size_t i = 0; i < lowerRingObstaclePositions.size(); ++i) {
        obstacles.emplace_back("sphere", lowerRingObstaclePositions[i], 0.15f);
    }
    
    // Add upper ring obstacles
    for (size_t i = 0; i < upperRingObstaclePositions.size(); ++i) {
        obstacles.emplace_back("sphere", upperRingObstaclePositions[i], 0.15f);
    }
    
    config.obstacles = std::move(obstacles);
    
    // Output settings
    config.save_path = true;
    config.description = " Panda 7-DOF Sphere Cage Navigation ";
    
    return config;
}

PlanningConfiguration create_planar_arm_example() {
    std::cout << " Creating 2DOF Planar Arm configuration..." << std::endl;
    
    PlanningConfiguration config;
    
    // Robot configuration
    config.robot_name = "planar_arm_2dof";
    config.description = "2DOF Planar Arm Navigation Demo";
    
    // Start configuration: arm pointing up and to the right (clear of obstacles)
    config.start_config = {
        0.7854f,   // shoulder: 45° (pointing up and right)
        0.0f       // elbow: 0° (straight)
    };
    
    // Goal configuration: arm pointing down and to the left
    config.goal_config = {
        -0.7854f,  // shoulder: -45° (pointing down and left)
        0.0f       // elbow: 0° (straight)
    };
    
    // Planning configuration
    config.planning.planner_name = "RRT-Connect";
    config.planning.planning_time = 5.0;  // Increased planning time
    config.planning.simplification_time = 1.0;
    config.planning.optimize_path = false;
    
    // Add planner-specific parameters to match YAML configuration
    config.planning.planner_parameters["range"] = "0.3";
    config.planning.planner_parameters["intermediate_states"] = "false";
    
    // Environment: Simple obstacle course
    std::cout << "Creating obstacle course for planar arm..." << std::endl;
    
    // Start with a very simple obstacle setup to ensure start/goal states are valid
    // Obstacle 1: Small sphere far from the robot's workspace
    ObstacleConfig obstacle1;
    obstacle1.type = "sphere";
    obstacle1.name = "far_obstacle";
    obstacle1.position = {0.8f, 0.0f, 0.0f};  // Far away from robot
    obstacle1.radius = 0.05f;
    config.obstacles.push_back(obstacle1);
    
    config.save_path = true;
    
    return config;
}