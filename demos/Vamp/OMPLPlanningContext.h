#pragma once

#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include "VampUtils.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
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
 * Design Patterns:
 * - Adapter Pattern: Adapts OMPL's interface to our unified planning interface
 * - Factory Pattern: Creates planners dynamically based on string identifiers
 * - Template Method: Standardizes the planning workflow
 * - Composition: Composes OMPL components rather than inheriting from them
 * 
 * Key Responsibilities:
 * - Setting up OMPL state spaces and bounds from robot configurations
 * - Creating and configuring planners using the Factory pattern
 * - Managing problem definitions and optimization objectives
 * - Running planning and simplification with comprehensive timing
 * - Providing clean separation between VAMP and OMPL concerns
 * 
 * Performance Considerations:
 * - Lazy planner creation: Planners created only when needed
 * - Reusable state spaces: Avoid recreation for multiple planning queries
 * - Efficient timing: High-resolution timing for performance analysis
 * - Memory management: Proper RAII for OMPL objects
 */
template<typename Robot>
class OMPLPlanningContext {
public:
    using RobotConfiguration = typename Robot::Configuration;
    static constexpr std::size_t robot_dimension_ = Robot::dimension;
    static constexpr std::size_t simd_lane_width_ = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simd_lane_width_>>;

private:
    std::shared_ptr<ob::SpaceInformation> space_information_;
    std::shared_ptr<ob::ProblemDefinition> problem_definition_;
    
public:
    /**
     * @brief Setup the OMPL state space using robot configuration
     * @param robot_configuration Robot configuration providing joint limits
     * @param vectorized_environment VAMP environment for validators
     * 
     * Note: This method demonstrates how to bridge different libraries'
     * configuration systems. Robot limits from VAMP are translated to OMPL's
     * bounds system, maintaining type safety and validation throughout.
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
        
        // Create space information
        space_information_ = std::make_shared<ob::SpaceInformation>(real_vector_space);
        
        // Set VAMP validators
        space_information_->setStateValidityChecker(
            std::make_shared<VampStateValidator<Robot>>(space_information_, vectorized_environment));
        space_information_->setMotionValidator(
            std::make_shared<VampMotionValidator<Robot>>(space_information_, vectorized_environment));
        
        space_information_->setup();
    }
    
    // Legacy method name for backward compatibility
    void setupStateSpace(const RobotConfig<Robot>& robot_configuration, 
                        const VectorizedEnvironment& vectorized_environment) {
        setup_state_space(robot_configuration, vectorized_environment);
    }
    
    /**
     * @brief Set up the planning problem with start and goal configurations
     * @param start_configuration Start configuration as array
     * @param goal_configuration Goal configuration as array
     * 
     * Note: This method shows how to convert between different
     * configuration representations while maintaining numerical precision.
     * The conversion from arrays to OMPL states is explicit and type-safe.
     */
    void set_problem(const std::array<float, robot_dimension_>& start_configuration, 
                    const std::array<float, robot_dimension_>& goal_configuration) {
        if (!space_information_) {
            throw VampConfigurationError("State space not set up. Call setup_state_space first.");
        }
        
        const auto& configuration_space = space_information_->getStateSpace();
        ob::ScopedState<> start_state_ompl(configuration_space);
        ob::ScopedState<> goal_state_ompl(configuration_space);
        
        // Convert arrays to OMPL states directly
        for (size_t joint_index = 0; joint_index < robot_dimension_; ++joint_index) {
            start_state_ompl[joint_index] = static_cast<double>(start_configuration[joint_index]);
            goal_state_ompl[joint_index] = static_cast<double>(goal_configuration[joint_index]);
        }
        
        // Create problem definition
        problem_definition_ = std::make_shared<ob::ProblemDefinition>(space_information_);
        problem_definition_->setStartAndGoalStates(start_state_ompl, goal_state_ompl);
        
        // Set optimization objective
        auto path_length_objective = std::make_shared<ob::PathLengthOptimizationObjective>(space_information_);
        problem_definition_->setOptimizationObjective(path_length_objective);
    }
    
    // Legacy method name for backward compatibility
    void setProblem(const std::array<float, robot_dimension_>& start_configuration, 
                   const std::array<float, robot_dimension_>& goal_configuration) {
        set_problem(start_configuration, goal_configuration);
    }
    
    /**
     * @brief Plan using the specified configuration
     * @param planningConfiguration Planning configuration
     * @return Planning results with timing and path information
     * 
     * Note: This method demonstrates the Template Method pattern -
     * it defines a standard planning workflow that works with any OMPL planner.
     * The high-resolution timing provides detailed performance metrics for analysis.
     */
    auto plan(const PlanningConfig& planning_configuration) -> PlanningResult {
        if (!problem_definition_) {
            throw VampConfigurationError("Problem not defined. Call set_problem first.");
        }
        
        PlanningResult planning_result;
        
        try {
            // Create planner
            auto selected_planner = create_planner_by_name(planning_configuration.planner_name);
            
            // Set planner parameters
            for (const auto& [param_name, param_value] : planning_configuration.planner_parameters) {
                bool success = selected_planner->params().setParam(param_name, param_value);
                (void)success; // Suppress unused variable warning
            }
            
            selected_planner->setProblemDefinition(problem_definition_);
            selected_planner->setup();
            
            // Plan with timing
            auto planning_start_time = std::chrono::steady_clock::now();
            ob::PlannerStatus planning_status = selected_planner->solve(planning_configuration.planning_time);
            auto planning_end_time = std::chrono::steady_clock::now();
            
            planning_result.planning_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                planning_end_time - planning_start_time).count();
            
            // Check if solution was found
            if (planning_status == ob::PlannerStatus::EXACT_SOLUTION) {
                planning_result.success = true;
                planning_result.solution_path = problem_definition_->getSolutionPath();
                
                // Get initial cost
                auto& geometric_path = static_cast<og::PathGeometric&>(*planning_result.solution_path);
                auto optimization_objective = problem_definition_->getOptimizationObjective();
                planning_result.initial_cost = geometric_path.cost(optimization_objective).value();
                planning_result.path_length = geometric_path.getStateCount();
                
                // Simplify path if requested
                if (planning_configuration.simplification_time > 0.0) {
                    auto simplification_start_time = std::chrono::steady_clock::now();
                    
                    og::PathSimplifier path_simplifier(space_information_, 
                                                     problem_definition_->getGoal(), 
                                                     optimization_objective);
                    bool path_was_simplified = path_simplifier.simplify(geometric_path, 
                                                                       planning_configuration.simplification_time);
                    
                    auto simplification_end_time = std::chrono::steady_clock::now();
                    planning_result.simplification_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        simplification_end_time - simplification_start_time).count();
                    
                    if (path_was_simplified) {
                        planning_result.final_cost = geometric_path.cost(optimization_objective).value();
                        planning_result.path_length = geometric_path.getStateCount();
                    } else {
                        planning_result.final_cost = planning_result.initial_cost;
                    }
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
        return space_information_;
    }
    
    /**
     * @brief Get the problem definition
     * @return Shared pointer to problem definition
     */
    auto get_problem_definition() const -> std::shared_ptr<ob::ProblemDefinition> {
        return problem_definition_;
    }
    
    // Legacy method names for backward compatibility
    std::shared_ptr<ob::SpaceInformation> getSpaceInformation() const {
        return get_space_information();
    }
    
    std::shared_ptr<ob::ProblemDefinition> getProblemDefinition() const {
        return get_problem_definition();
    }

private:
    /**
     * @brief Extensible Factory pattern for planner creation with Registry approach
     * 
     * This factory implements the Open/Closed Principle by allowing runtime
     * registration of new planners without source code modification.
     * Uses the Registry pattern with lambda-based factory functions.
     */
    class PlannerFactory {
    public:
        using PlannerCreatorFunction = std::function<std::shared_ptr<ob::Planner>(const ob::SpaceInformationPtr&)>;
        
        /**
         * @brief Get factory instance
         * 
         */
        static PlannerFactory& getInstance() {
            static PlannerFactory factoryInstance;
            return factoryInstance;
        }
        
        /**
         * @brief Create planner by name using registry lookup
         * @param plannerName Planner name as string identifier
         * @param spaceInformation Space information for planner initialization
         * @return Created planner instance
         * @throws VampConfigurationError for unknown planners (explicit error handling)
         * 
         */
        std::shared_ptr<ob::Planner> createPlanner(const std::string& plannerName, 
                                                   const ob::SpaceInformationPtr& spaceInformation) {
            auto factoryIterator = m_plannerCreators.find(plannerName);
            if (factoryIterator == m_plannerCreators.end()) {
                throw VampConfigurationError("Unknown planner: '" + plannerName + "'. Available: " + 
                                          getAvailablePlannerNames());
            }
            return factoryIterator->second(spaceInformation);
        }
        
        /**
         * @brief Register a new planner type (Open/Closed Principle compliance)
         * @param planner_name Name identifier for the planner
         * @param plannerAllocatorFn Factory function to create the planner
         */
        void register_planner(const std::string& planner_name, PlannerCreatorFunction plannerAllocatorFn) {
            m_plannerCreators[planner_name] = std::move(plannerAllocatorFn);
        }
        
        /**
         * @brief Get list of available planners for error messages and user interfaces
         * 
         */
        std::string getAvailablePlannerNames() const {
            std::string plannerNamesList;
            for (const auto& plannerEntry : m_plannerCreators) {
                if (!plannerNamesList.empty()) plannerNamesList += ", ";
                plannerNamesList += plannerEntry.first;
            }
            return plannerNamesList;
        }
        
    private:
        std::map<std::string, PlannerCreatorFunction> m_plannerCreators;
        
        /**
         * @brief Constructor registers default OMPL planners using lambda functions
         * 
         * This constructor only registers the core OMPL planners that are commonly used.
         * Additional planners can be registered at runtime using the register_planner() 
         * method without modifying this source code.
         * 
         * Extensibility Pattern:
         * The factory follows the Open/Closed Principle:
         * - OPEN for extension: New planners can be added via registerPlanner()
         * - CLOSED for modification: No source code changes needed for new planners
         * 
         * Example of runtime planner registration:
         * ```cpp
         * OMPLPlanningContext<Robot>::registerPlanner("RRT*", 
         *     [](const ob::SpaceInformationPtr& si) {
         *         return std::make_shared<og::RRTstar>(si);
         *     });
         * ```
         * 
         * Performance Note: Lambda functions are compiled to optimal code
         * with no runtime overhead compared to function pointers.
         */
        PlannerFactory() {
            // Register standard OMPL planners with descriptive names
            m_plannerCreators["BIT*"] = [](const ob::SpaceInformationPtr& spaceInformation) {
                return std::make_shared<og::BITstar>(spaceInformation);
            };
            m_plannerCreators["RRT-Connect"] = [](const ob::SpaceInformationPtr& spaceInformation) {
                return std::make_shared<og::RRTConnect>(spaceInformation);
            };
            m_plannerCreators["PRM"] = [](const ob::SpaceInformationPtr& spaceInformation) {
                return std::make_shared<og::PRM>(spaceInformation);
            };
            
            // Note: Additional planners can be registered at runtime using:
            // OMPLPlanningContext<Robot>::registerPlanner(name, factory_function)
            // This eliminates the need to modify this source code for new planners.
        }
    };
    
    /**
     * @brief Register a new planner type for runtime extensibility (Open/Closed Principle)
     * 
     * This method allows adding new OMPL planners without modifying the library source code.
     * The factory function will be called each time a planner instance is needed.
     * 
     * @param planner_name Unique string identifier for the planner
     * @param plannerAllocatorFn Factory function that creates planner instances
     * 
     * Example:
     * ```cpp
     * OMPLPlanningContext<Robot>::registerPlanner("RRT*", 
     *     [](const ob::SpaceInformationPtr& si) {
     *         auto planner = std::make_shared<og::RRTstar>(si);
     *         // Optional: Set planner-specific parameters here
     *         return planner;
     *     });
     * ```
     * 
     * @note Thread safety: Registration should typically occur during initialization
     * before concurrent planning operations begin.
     */
    static void registerPlanner(const std::string& planner_name, 
                               std::function<std::shared_ptr<ob::Planner>(const ob::SpaceInformationPtr&)> plannerAllocatorFn) {
        PlannerFactory::getInstance().register_planner(planner_name, std::move(plannerAllocatorFn));
    }
    
    /**
     * @brief Create planner by name
     * 
     * Note: This method demonstrates the Facade pattern applied to
     * factory access. It hides the complexity of factory singleton access and
     * provides a clean, simple interface for planner creation.
     */
    auto create_planner_by_name(const std::string& planner_name) -> std::shared_ptr<ob::Planner> {
        return PlannerFactory::getInstance().createPlanner(planner_name, space_information_);
    }
    
    // Legacy method name for backward compatibility  
    std::shared_ptr<ob::Planner> createPlannerByName(const std::string& planner_name) {
        return create_planner_by_name(planner_name);
    }
    

};

} // namespace vamp_ompl