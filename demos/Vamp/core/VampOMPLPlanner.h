#pragma once

#include "VampOMPLInterfaces.h"
#include "OMPLPlanningContext.h"
#include "VampUtils.h"
#include <ompl/geometric/PathGeometric.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <filesystem>
#include <algorithm>

namespace vamp_ompl {

/**
 * @brief Main VAMP-OMPL integration planner
 * 
 * This class acts as a unified interface that coordinates between the VAMP collision
 * detection system and OMPL's motion planning algorithms. It implements the Facade
 * design pattern to hide the complexity of integrating these two systems.
 * 
 * @tparam Robot VAMP robot type (e.g., vamp::robots::Panda)
 */
template<typename Robot>
class VampOMPLPlanner {
public:
    using RobotConfiguration = typename Robot::Configuration;
    static constexpr std::size_t robot_dimension_ = Robot::dimension;
    static constexpr std::size_t simd_lane_width_ = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simd_lane_width_>>;

private:
    std::unique_ptr<RobotConfig<Robot>> robot_configuration_;
    std::unique_ptr<EnvironmentFactory> environment_factory_;
    OMPLPlanningContext<Robot> planning_context_;
    VectorizedEnvironment vectorized_environment_;
    bool is_initialized_;

public:
    /**
     * @brief Constructor taking robot and environment configuration (Dependency Injection)
     * @param robot_configuration Robot configuration providing limits and poses
     * @param environment_factory Factory for creating collision environments
     * 
     * Note: This constructor demonstrates dependency injection - rather than
     * creating its dependencies internally, the planner accepts them as parameters.
     * This design promotes testability, flexibility, and separation of concerns.
     */
    VampOMPLPlanner(std::unique_ptr<RobotConfig<Robot>> robot_configuration,
                    std::unique_ptr<EnvironmentFactory> environment_factory)
        : robot_configuration_(std::move(robot_configuration))
        , environment_factory_(std::move(environment_factory))
        , is_initialized_(false) {
        
        if (!robot_configuration_) {
            throw VampConfigurationError("Robot configuration cannot be null");
        }
        if (!environment_factory_) {
            throw VampConfigurationError("Environment factory cannot be null");
        }
    }

    /**
     * @brief Initialize the planner (create environment, setup OMPL)
     * This must be called before planning
     * 
     * Note: This two-phase initialization pattern (constructor + initialize)
     * is common in complex systems where initialization can fail or is expensive.
     * It allows error handling and provides clear separation between object creation
     * and system setup.
     * 
     * Performance Note: Environment vectorization happens here, converting scalar
     * collision geometry to SIMD-optimized format for speedup.
     */
    void initialize() {   
        // Create collision environment
        auto scalar_environment = environment_factory_->create_environment();
        vectorized_environment_ = VectorizedEnvironment(scalar_environment);
        
        // Setup OMPL state space and validators
        planning_context_.setup_state_space(*robot_configuration_, vectorized_environment_);
        
        // Setup default problem (start and goal from robot config)
        auto default_start_configuration = robot_configuration_->get_start_configuration_array();
        auto default_goal_configuration = robot_configuration_->get_goal_configuration_array();
        planning_context_.set_problem(default_start_configuration, default_goal_configuration);
        
        is_initialized_ = true;
        std::cout << "Initialization complete" << std::endl;
    }
    
    /**
     * @brief Unified planning function - works with robot's default or custom start/goal
     * @param planning_configuration Planning configuration (time limits, planner type, etc.)
     * @param custom_start_configuration Optional custom start configuration (if empty, uses robot's default)
     * @param custom_goal_configuration Optional custom goal configuration (if empty, uses robot's default)
     * @return Planning results with comprehensive timing and path information
     * 
     * Note: This method demonstrates the Template Method pattern - it
     * defines a standard algorithm structure (validate → configure → plan → process)
     * while allowing customization through parameters.
     * 
     * Performance Note: Configuration validation happens at planning time rather than
     * construction time, allowing for dynamic reconfiguration without object recreation.
     */
    auto plan(const PlanningConfig& planning_configuration = PlanningConfig(),
             const std::array<float, robot_dimension_>& custom_start_configuration = {},
             const std::array<float, robot_dimension_>& custom_goal_configuration = {}) -> PlanningResult {
        
        if (!is_initialized_) {
            throw VampConfigurationError("Planner not initialized. Call initialize() first.");
        }
        
        // Use custom start/goal if provided, otherwise use robot's defaults
        bool use_custom_configurations = (custom_start_configuration != std::array<float, robot_dimension_>{});
        if (use_custom_configurations) {
            planning_context_.set_problem(custom_start_configuration, custom_goal_configuration);
        }
        
        auto planning_result = planning_context_.plan(planning_configuration);
        
        // Write solution path using OMPL's built-in functionality if requested
        if (planning_configuration.write_path && planning_result.success && planning_result.solution_path) {
            planning_result.solution_file_path = write_optimized_solution_path(
                planning_result, planning_configuration.planner_name);
        }
        
        return planning_result;
    }

    /**
     * @brief Write solution path using OMPL's built-in printAsMatrix() functionality
     * 
     * File Format: The output format is compatible with standard robotics
     * visualization tools and can be easily imported into Python/MATLAB for analysis.
     */
    auto write_optimized_solution_path(const PlanningResult& planning_result, 
                                      const std::string& planner_name, 
                                      const PlanningConfiguration::VisualizationConfig& visualization_config = {}) const -> std::string
    {
        auto geometric_path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(planning_result.solution_path);
        if (!planning_result.success || !geometric_path) return "";
        
        geometric_path->interpolate(constants::DEFAULT_PATH_WAYPOINTS); // Standard waypoint count
        
        // Find output directory and create filename
        auto find_output_directory = [](const std::vector<std::string>& candidate_paths) {
            for (const auto& path_candidate : candidate_paths) {
                if (std::filesystem::exists(path_candidate)) return path_candidate;
            }
            return std::string("./");
        };
        
        // Clean names for filename (replace spaces with underscores)
        auto clean_name = [](const std::string& name) {
            std::string cleaned = name;
            std::replace(cleaned.begin(), cleaned.end(), ' ', '_');
            return cleaned;
        };
        
        std::string output_filename = find_output_directory({"demos/Vamp/", "../demos/Vamp/", "../../demos/Vamp/"}) + 
                              "solution_path_" + clean_name(robot_configuration_->get_robot_name()) + "_" + 
                              clean_name(environment_factory_->get_environment_name()) + "_" + clean_name(planner_name) + ".txt";
        
        std::ofstream output_file(output_filename);
        if (!output_file) return "";
        
        // Write header using compact format
        output_file << "# " << robot_configuration_->get_robot_name() << " + " << environment_factory_->get_environment_name() 
             << " + " << planner_name << " (dim=" << robot_dimension_ << ", waypoints=" 
             << geometric_path->getStateCount() << ", cost=" << planning_result.final_cost << ")\n";
        
        // Write visualization configuration if provided
        if (!visualization_config.urdf_path.empty()) {
            output_file << "# VISUALIZATION CONFIG:\n";
            output_file << "# robot_name: " << robot_configuration_->get_robot_name() << "\n";
            output_file << "# urdf_path: " << visualization_config.urdf_path << "\n";
            if (visualization_config.expected_joints > 0) {
                output_file << "# expected_joints: " << visualization_config.expected_joints << "\n";
            }
            output_file << "# base_position: [" << visualization_config.base_position[0] << ", " 
                      << visualization_config.base_position[1] << ", " << visualization_config.base_position[2] << "]\n";
            output_file << "# base_orientation: [" << visualization_config.base_orientation[0] << ", " 
                      << visualization_config.base_orientation[1] << ", " << visualization_config.base_orientation[2] << "]\n";
            output_file << "# use_fixed_base: " << (visualization_config.use_fixed_base ? "true" : "false") << "\n";
        }
        
        // Use OMPL's built-in path writing functionality
        geometric_path->printAsMatrix(output_file);
        
        std::cout << " Path written: " << output_filename << " (" << geometric_path->getStateCount() 
                  << " waypoints, cost=" << planning_result.final_cost << ")" << std::endl;
        return output_filename;
    }
    
    /**
     * @brief Get robot configuration
     * @return Reference to robot configuration
     */
    auto get_robot_configuration() const -> const RobotConfig<Robot>& {
        return *robot_configuration_;
    }
    
    /**
     * @brief Get environment factory
     * @return Reference to environment factory
     */
    auto get_environment_factory() const -> const EnvironmentFactory& {
        return *environment_factory_;
    }
    
    /**
     * @brief Get OMPL planning context
     * @return Reference to OMPL planning context
     */
    auto get_planning_context() const -> const OMPLPlanningContext<Robot>& {
        return planning_context_;
    }
    
    /**
     * @brief Check if planner is initialized
     * @return true if initialized, false otherwise
     */
    auto is_initialized() const -> bool {
        return is_initialized_;
    }
    
    /**
     * @brief Get the OMPL space information for advanced use cases
     * @return Shared pointer to the configured space information
     * 
     * Note: This method provides controlled access to the underlying OMPL setup
     * for scenarios like benchmarking that need direct access to the space information.
     * This follows the principle of exposing minimal necessary interface rather than
     * duplicating setup logic.
     */
    auto get_space_information() const -> std::shared_ptr<ompl::base::SpaceInformation> {
        if (!is_initialized_) {
            throw VampConfigurationError("Planner not initialized. Call initialize() first.");
        }
        return planning_context_.get_space_information();
    }
    
    /**
     * @brief Print configuration summary
     */
    void print_configuration() const {
        VampUtils::printPlannerConfiguration<Robot>(*robot_configuration_, *environment_factory_, is_initialized_);
    }

};

/**
 * @brief Convenience function to create a VAMP-OMPL planner (Factory Function)
 * @param robot_configuration Robot configuration
 * @param environment_factory Environment factory
 * @return Unique pointer to configured planner
 * 
 * Note: This factory function encapsulates object creation logic,
 * making client code cleaner and providing a stable interface even if the
 * constructor signature changes in the future.
 */
template<typename Robot>
auto create_vamp_ompl_planner(std::unique_ptr<RobotConfig<Robot>> robot_configuration,
                             std::unique_ptr<EnvironmentFactory> environment_factory) 
    -> std::unique_ptr<VampOMPLPlanner<Robot>> {
    
    return std::make_unique<VampOMPLPlanner<Robot>>(
        std::move(robot_configuration), std::move(environment_factory));
}

} // namespace vamp_ompl