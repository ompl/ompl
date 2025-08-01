#include "VampOMPLDemo.h"
#include "VampUtils.h"
#include <iostream>
#include <string>

using namespace vamp_ompl;

/**
 * @brief VAMP-OMPL integration demonstration showcasing vectorized motion planning
 * 
 * This demo showcases VAMP's SIMD-accelerated collision checking with OMPL planners.
 * 
 * Usage:
 *   ./VampDemo                    # Run basic programmatic example
 *   ./VampDemo --visualize        # Run basic example + visualization  
 *   ./VampDemo panda_demo.yaml    # Run YAML configuration example
 * 
 */

/**
 * @brief Create the basic setup example configuration
 * 
 *  Note: This function demonstrates how to create motion planning
 * configurations directly in code. This approach is useful for research applications
 * where configurations need to be generated algorithmically or for batch processing.
 * 
 * The configuration showcases a typical pick-and-place scenario with a Panda robot
 * navigating through a sphere cage environment.
 */
PlanningConfiguration createBasicExample()
{
    std::cout << "🔧 Creating basic setup configuration..." << std::endl;
    
    // Step 1: Create configuration object
    PlanningConfiguration planningConfiguration;
    
    // Step 2: Configure robot
    planningConfiguration.robot_name = "panda";
    planningConfiguration.description = "Basic Panda Sphere Cage Demo";
    
    // Step 3: Set explicit start configuration (joint angles in radians)
    //  Note: These joint angles represent a typical "ready" pose
    // for manipulation tasks - arm slightly bent, end-effector in workspace
    planningConfiguration.start_config = {
        0.0f,      // panda_joint1: Base rotation
        -0.785f,   // panda_joint2: Shoulder
        0.0f,      // panda_joint3: Upper arm rotation
        -2.356f,   // panda_joint4: Elbow 
        0.0f,      // panda_joint5: Forearm rotation
        1.571f,    // panda_joint6: Wrist pitch 
        0.785f     // panda_joint7: Wrist roll 
    };
    
    // Step 4: Set explicit goal configuration (joint angles in radians)
    //  Note: This represents a different manipulation pose,
    // requiring the robot to navigate around obstacles
    planningConfiguration.goal_config = {
        2.35f,     // panda_joint1: Base rotation
        1.0f,      // panda_joint2: Shoulder
        0.0f,      // panda_joint3: Upper arm rotation
        -0.8f,     // panda_joint4: Elbow
        0.0f,      // panda_joint5: Forearm rotation
        2.5f,      // panda_joint6: Wrist pitch
        0.785f     // panda_joint7: Wrist roll
    };
    
    // Step 5: Configure planner
    planningConfiguration.planning.planner_name = "RRT-Connect";
    planningConfiguration.planning.planning_time = 2.0;
    planningConfiguration.planning.simplification_time = 1.0;
    planningConfiguration.planning.optimize_path = false;
    planningConfiguration.save_path = true;
    
    // Step 6: Create explicit environment - Sphere cage with two rings
    //  Note: This creates a challenging but solvable environment
    // that demonstrates the planner's ability to navigate complex obstacle fields
    std::cout << "Creating sphere cage environment with explicit obstacle definitions..." << std::endl;
    
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
    
    // Add lower ring spheres
    for (size_t obstacleIndex = 0; obstacleIndex < lowerRingObstaclePositions.size(); ++obstacleIndex) {
        ObstacleConfig sphereObstacle;
        sphereObstacle.type = "sphere";
        sphereObstacle.name = "sphere_lower_" + std::to_string(obstacleIndex);
        sphereObstacle.position = lowerRingObstaclePositions[obstacleIndex];
        sphereObstacle.radius = 0.15f;
        planningConfiguration.obstacles.push_back(sphereObstacle);
    }
    
    // Add upper ring spheres
    for (size_t obstacleIndex = 0; obstacleIndex < upperRingObstaclePositions.size(); ++obstacleIndex) {
        ObstacleConfig sphereObstacle;
        sphereObstacle.type = "sphere";
        sphereObstacle.name = "sphere_upper_" + std::to_string(obstacleIndex);
        sphereObstacle.position = upperRingObstaclePositions[obstacleIndex];
        sphereObstacle.radius = 0.15f;
        planningConfiguration.obstacles.push_back(sphereObstacle);
    }
    
    std::cout << "✓ Created " << planningConfiguration.obstacles.size() << " obstacles directly in code" << std::endl;
    
    return planningConfiguration;
}

/**
 * @brief Load configuration from YAML file (using utilities)
 * 
 */
PlanningConfiguration loadYamlExample(const std::string& yamlConfigurationFile)
{
    PlanningConfiguration yamlPlanningConfiguration;
    if (!VampUtils::loadYamlConfig(yamlConfigurationFile, yamlPlanningConfiguration)) {
        throw VampYamlError("Failed to load YAML configuration: " + yamlConfigurationFile);
    }
    yamlPlanningConfiguration.save_path = true; // Always save path for potential visualization
    return yamlPlanningConfiguration;
}




int main(int argc, char **argv)
{
    try {
        bool shouldRunVisualization = false;
        std::string yamlConfigurationFile;
        
        // Parse arguments
        if (argc == 1) {
            // Default: basic programmatic example
            // No additional flags needed
        } else if (argc == 2) {
            std::string commandLineArgument = argv[1];
            if (commandLineArgument == "--help" || commandLineArgument == "-h") {
                VampUtils::printUsage(argv[0]);
                return 0;
            } else if (commandLineArgument == "--visualize") {
                shouldRunVisualization = true;
            } else if (commandLineArgument.size() >= 5 && commandLineArgument.substr(commandLineArgument.size() - 5) == ".yaml") {
                yamlConfigurationFile = commandLineArgument;
            } else {
                std::cout << "❌ Unknown argument: " << commandLineArgument << std::endl;
                VampUtils::printUsage(argv[0]);
                return 1;
            }
        } else {
            std::cout << "❌ Too many arguments" << std::endl;
            VampUtils::printUsage(argv[0]);
            return 1;
        }
        
        MotionPlanningResult motionPlanningResult;
        std::string configurationSource;
        
        if (!yamlConfigurationFile.empty()) {
            // YAML mode - demonstrates configuration-driven planning
            std::cout << "Mode: YAML Configuration" << std::endl;
            auto yamlPlanningConfiguration = loadYamlExample(yamlConfigurationFile);
            VampUtils::printConfigSummary(yamlPlanningConfiguration);
            
            std::cout << "🚀 Starting planning with YAML configuration..." << std::endl;
            motionPlanningResult = executeMotionPlanning(yamlPlanningConfiguration);
            configurationSource = yamlConfigurationFile;
            shouldRunVisualization = true; // Always offer visualization for YAML mode
            
        } else {
            // Basic setup mode - demonstrates code-based configuration
            std::cout << "Mode: Basic Setup Configuration" << std::endl;
            auto basicPlanningConfiguration = createBasicExample();
            
            // Validate configuration
            if (!basicPlanningConfiguration.isValid()) {
                std::cout << "❌ Configuration validation failed: " << basicPlanningConfiguration.getValidationErrors() << std::endl;
                return 1;
            }
            
            std::cout << "✓ Configuration validation passed" << std::endl;
            std::cout << "🚀 Starting planning with basic setup configuration..." << std::endl;
            motionPlanningResult = executeMotionPlanning(basicPlanningConfiguration);
            configurationSource = "basic_setup";
        }
        
        // Show results
        VampUtils::printResults(motionPlanningResult);
        
        if (motionPlanningResult.success()) {
            // Handle visualization
            if (shouldRunVisualization) {
                std::cout << "\n🎬 Starting visualization..." << std::endl;
                if (!VampUtils::runVisualization(motionPlanningResult, configurationSource)) {
                    std::cout << "⚠️  Visualization failed (planning still succeeded)" << std::endl;
                }
            } else {
                std::cout << "\n💡 To visualize this result, run:" << std::endl;
                std::cout << "   " << argv[0] << " --visualize" << std::endl;
            }
        } else {
            return 1;
        }
        
        return 0;
        
    } catch (const std::exception& caughtException) {
        std::cerr << "❌ Fatal error: " << caughtException.what() << std::endl;
        return 1;
    }
} 