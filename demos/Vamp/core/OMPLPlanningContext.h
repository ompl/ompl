#pragma once

#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include "VampUtils.h"
#include "OMPLPlannerRegistry.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <chrono>
#include <iostream>
#include <functional>
#include <map>

namespace vamp_ompl {

namespace og = ompl::geometric;

/**
 * @brief OMPL planning context that manages OMPL-specific setup and planning
 * 
 * This class encapsulates all OMPL-specific functionality and serves as an adapter
 * between the generic VAMP-OMPL interface and OMPL's internal structures.
 * 
 * @tparam Robot VAMP robot type (e.g., vamp::robots::Panda)
 */
template<typename Robot>
class OMPLPlanningContext {
public:
    using RobotConfiguration = typename Robot::Configuration;
    static constexpr std::size_t robot_dimension_ = Robot::dimension;
    static constexpr std::size_t simd_lane_width_ = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simd_lane_width_>>;

private:
    std::unique_ptr<og::SimpleSetup> simple_setup_;
    
public:
    /**
     * @brief Setup the OMPL state space using robot configuration
     * @param robot_configuration Robot configuration providing joint limits
     * @param vectorized_environment VAMP environment for validators
     */
    void setup_state_space(const RobotConfig<Robot>& robot_configuration, 
                          const VectorizedEnvironment& vectorized_environment) {
        // Create state space
        auto real_vector_space = std::make_shared<ob::RealVectorStateSpace>(robot_dimension_);
        
        // Set joint limits
        ob::RealVectorBounds joint_bounds(robot_dimension_);
        auto robot_joint_limits = robot_configuration.get_joint_limits();
        
        if (robot_joint_limits.size() != robot_dimension_) {
            throw VampConfigurationError(
                "Joint limits size (" + std::to_string(robot_joint_limits.size()) + 
                ") does not match robot dimension (" + std::to_string(robot_dimension_) + ")");
        }
        
        for (size_t joint_index = 0; joint_index < robot_dimension_; ++joint_index) {
            joint_bounds.setLow(joint_index, robot_joint_limits[joint_index].first);
            joint_bounds.setHigh(joint_index, robot_joint_limits[joint_index].second);
        }
        
        real_vector_space->setBounds(joint_bounds);
        
        // Create SimpleSetup
        simple_setup_ = std::make_unique<og::SimpleSetup>(real_vector_space);
        
        // Set VAMP validators
        simple_setup_->setStateValidityChecker(
            std::make_shared<VampStateValidator<Robot>>(simple_setup_->getSpaceInformation(), vectorized_environment));
        simple_setup_->getSpaceInformation()->setMotionValidator(
            std::make_shared<VampMotionValidator<Robot>>(simple_setup_->getSpaceInformation(), vectorized_environment));
        
        // Set optimization objective
        auto path_length_objective = std::make_shared<ob::PathLengthOptimizationObjective>(simple_setup_->getSpaceInformation());
        simple_setup_->getProblemDefinition()->setOptimizationObjective(path_length_objective);
    }
    
    /**
     * @brief Set up the planning problem with start and goal configurations
     * @param start_configuration Start configuration as array
     * @param goal_configuration Goal configuration as array
     */
    void set_problem(const std::array<float, robot_dimension_>& start_configuration, 
                    const std::array<float, robot_dimension_>& goal_configuration) {
        if (!simple_setup_) {
            throw VampConfigurationError("State space not set up. Call setup_state_space first.");
        }
        
        const auto& configuration_space = simple_setup_->getStateSpace();
        ob::ScopedState<> start_state_ompl(configuration_space);
        ob::ScopedState<> goal_state_ompl(configuration_space);
        
        // Convert arrays to OMPL states directly
        for (size_t joint_index = 0; joint_index < robot_dimension_; ++joint_index) {
            start_state_ompl[joint_index] = static_cast<double>(start_configuration[joint_index]);
            goal_state_ompl[joint_index] = static_cast<double>(goal_configuration[joint_index]);
        }
        
        simple_setup_->setStartAndGoalStates(start_state_ompl, goal_state_ompl);
    }
    
    /**
     * @brief Plan using the specified configuration
     * @param planning_configuration Planning configuration
     * @return Planning results with timing and path information
     */
    auto plan(const PlanningConfig& planning_configuration) -> PlanningResult {
        if (!simple_setup_) {
            throw VampConfigurationError("Problem not defined. Call set_problem first.");
        }
        
        PlanningResult planning_result;
        
        try {
            // Set planner using OMPL's recommended setPlannerAllocator approach
            simple_setup_->setPlannerAllocator(create_planner_allocator(planning_configuration.planner_name, planning_configuration.planner_parameters));
            
            // Plan with timing
            auto planning_start_time = std::chrono::steady_clock::now();
            ob::PlannerStatus planning_status = simple_setup_->solve(planning_configuration.planning_time);
            auto planning_end_time = std::chrono::steady_clock::now();
            
            planning_result.planning_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                planning_end_time - planning_start_time).count();
            
            // Check if solution was found
            if (planning_status == ob::PlannerStatus::EXACT_SOLUTION) {
                planning_result.success = true;
                planning_result.solution_path = simple_setup_->getProblemDefinition()->getSolutionPath();
                
                // Get initial cost
                auto& geometric_path = static_cast<og::PathGeometric&>(*planning_result.solution_path);
                auto optimization_objective = simple_setup_->getProblemDefinition()->getOptimizationObjective();
                planning_result.initial_cost = geometric_path.cost(optimization_objective).value();
                planning_result.path_length = geometric_path.getStateCount();
                
                // Simplify path if requested
                if (planning_configuration.simplification_time > 0.0) {
                    auto simplification_start_time = std::chrono::steady_clock::now();
                    
                    simple_setup_->simplifySolution(planning_configuration.simplification_time);
                    
                    auto simplification_end_time = std::chrono::steady_clock::now();
                    planning_result.simplification_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        simplification_end_time - simplification_start_time).count();
                    
                    planning_result.final_cost = geometric_path.cost(optimization_objective).value();
                    planning_result.path_length = geometric_path.getStateCount();
                } else {
                    planning_result.final_cost = planning_result.initial_cost;
                }
            } else {
                planning_result.success = false;
                planning_result.error_message = "No solution found within time limit";
            }
            
        } catch (const std::exception& exception) {
            planning_result.success = false;
            planning_result.error_message = std::string("Planning failed: ") + exception.what();
        }
        
        return planning_result;
    }
    
    /**
     * @brief Get the space information
     * @return Shared pointer to space information
     */
    auto get_space_information() const -> std::shared_ptr<ob::SpaceInformation> {
        return simple_setup_ ? simple_setup_->getSpaceInformation() : nullptr;
    }
    
    /**
     * @brief Get the problem definition
     * @return Shared pointer to problem definition
     */
    auto get_problem_definition() const -> std::shared_ptr<ob::ProblemDefinition> {
        return simple_setup_ ? simple_setup_->getProblemDefinition() : nullptr;
    }

private:
    /**
     * @brief Create planner allocator using the registry pattern
     * @param planner_name Name of the planner to create
     * @param planner_parameters Parameters to set on the planner
     * @return Planner allocator function
     * 
     * This method uses the PlannerRegistry to create planners, allowing users
     * to register custom planners without modifying this code. The registry
     * handles parameter configuration using OMPL's ParamSet system.
     */
    auto create_planner_allocator(const std::string& planner_name, 
                                 const std::map<std::string, std::string>& planner_parameters) -> ob::PlannerAllocator {
        
        return [planner_name, planner_parameters](const ob::SpaceInformationPtr& si) -> ob::PlannerPtr {
            // Use the registry to create the planner with parameters
            return createPlannerByName(planner_name, si, planner_parameters);
        };
    }
};

} // namespace vamp_ompl