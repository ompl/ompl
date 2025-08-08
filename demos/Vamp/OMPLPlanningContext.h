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
    static constexpr std::size_t robotDimension = Robot::dimension;
    static constexpr std::size_t simdLaneWidth = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simdLaneWidth>>;

private:
    std::shared_ptr<ob::SpaceInformation> m_spaceInformation;
    std::shared_ptr<ob::ProblemDefinition> m_problemDefinition;
    
public:
    /**
     * @brief Setup the OMPL state space using robot configuration
     * @param robotConfiguration Robot configuration providing joint limits
     * @param vectorizedEnvironment VAMP environment for validators
     * 
     * Note: This method demonstrates how to bridge different libraries'
     * configuration systems. Robot limits from VAMP are translated to OMPL's
     * bounds system, maintaining type safety and validation throughout.
     */
    void setupStateSpace(const RobotConfig<Robot> &robotConfiguration, const VectorizedEnvironment &vectorizedEnvironment)
    {
        // Create state space
        auto realVectorSpace = std::make_shared<ob::RealVectorStateSpace>(robotDimension);
        
        // Set joint limits
        ob::RealVectorBounds jointBounds(robotDimension);
        auto robotJointLimits = robotConfiguration.getJointLimits();
        
        if (robotJointLimits.size() != robotDimension) {
            throw VampConfigurationError("Joint limits size (" + std::to_string(robotJointLimits.size()) + 
                                       ") does not match robot dimension (" + std::to_string(robotDimension) + ")");
        }
        
        for (size_t jointIndex = 0; jointIndex < robotDimension; ++jointIndex) {
            jointBounds.setLow(jointIndex, robotJointLimits[jointIndex].first);
            jointBounds.setHigh(jointIndex, robotJointLimits[jointIndex].second);
        }
        
        realVectorSpace->setBounds(jointBounds);
        
        // Create space information
        m_spaceInformation = std::make_shared<ob::SpaceInformation>(realVectorSpace);
        
        // Set VAMP validators
        m_spaceInformation->setStateValidityChecker(std::make_shared<VampStateValidator<Robot>>(m_spaceInformation, vectorizedEnvironment));
        m_spaceInformation->setMotionValidator(std::make_shared<VampMotionValidator<Robot>>(m_spaceInformation, vectorizedEnvironment));
        
        m_spaceInformation->setup();
    }
    
    /**
     * @brief Set up the planning problem with start and goal configurations
     * @param startConfiguration Start configuration as array
     * @param goalConfiguration Goal configuration as array
     * 
     * Note: This method shows how to convert between different
     * configuration representations while maintaining numerical precision.
     * The conversion from arrays to OMPL states is explicit and type-safe.
     */
    void setProblem(const std::array<float, robotDimension> &startConfiguration, 
                   const std::array<float, robotDimension> &goalConfiguration)
    {
        if (!m_spaceInformation) {
            throw VampConfigurationError("State space not set up. Call setupStateSpace first.");
        }
        
        auto configurationSpace = m_spaceInformation->getStateSpace();
        ob::ScopedState<> startStateOmpl(configurationSpace), goalStateOmpl(configurationSpace);
        
        // Convert arrays to OMPL states directly
        for (size_t jointIndex = 0; jointIndex < robotDimension; ++jointIndex) {
            startStateOmpl[jointIndex] = static_cast<double>(startConfiguration[jointIndex]);
            goalStateOmpl[jointIndex] = static_cast<double>(goalConfiguration[jointIndex]);
        }
        
        // Create problem definition
        m_problemDefinition = std::make_shared<ob::ProblemDefinition>(m_spaceInformation);
        m_problemDefinition->setStartAndGoalStates(startStateOmpl, goalStateOmpl);
        
        // Set optimization objective
        auto pathLengthObjective = std::make_shared<ob::PathLengthOptimizationObjective>(m_spaceInformation);
        m_problemDefinition->setOptimizationObjective(pathLengthObjective);
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
    PlanningResult plan(const PlanningConfig &planningConfiguration)
    {
        if (!m_problemDefinition) {
            throw VampConfigurationError("Problem not defined. Call setProblem first.");
        }
        
        PlanningResult planningResult;
        
        try {
            // Create planner
            auto selectedPlanner = createPlannerByName(planningConfiguration.planner_name);
            
            // Set planner parameters
            for (const auto& [param_name, param_value] : planningConfiguration.planner_parameters) {
                bool success = selectedPlanner->params().setParam(param_name, param_value);
            }
            
            selectedPlanner->setProblemDefinition(m_problemDefinition);
            selectedPlanner->setup();
            
            // Plan
            auto planningStartTime = std::chrono::steady_clock::now();
            ob::PlannerStatus planningStatus = selectedPlanner->solve(planningConfiguration.planning_time);
            auto planningEndTime = std::chrono::steady_clock::now();
            
            planningResult.planning_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                planningEndTime - planningStartTime).count();
            
            // Check if solution was found
            if (planningStatus == ob::PlannerStatus::EXACT_SOLUTION) {
                planningResult.success = true;
                planningResult.solution_path = m_problemDefinition->getSolutionPath();
                
                // Get initial cost
                og::PathGeometric &geometricPath = static_cast<og::PathGeometric &>(*planningResult.solution_path);
                auto optimizationObjective = m_problemDefinition->getOptimizationObjective();
                planningResult.initial_cost = geometricPath.cost(optimizationObjective).value();
                planningResult.path_length = geometricPath.getStateCount();
                
                // Simplify path if requested
                if (planningConfiguration.simplification_time > 0.0) {
                    auto simplificationStartTime = std::chrono::steady_clock::now();
                    
                    og::PathSimplifier pathSimplifier(m_spaceInformation, m_problemDefinition->getGoal(), optimizationObjective);
                    bool pathWasSimplified = pathSimplifier.simplify(geometricPath, planningConfiguration.simplification_time);
                    
                    auto simplificationEndTime = std::chrono::steady_clock::now();
                    planningResult.simplification_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        simplificationEndTime - simplificationStartTime).count();
                    
                    if (pathWasSimplified) {
                        planningResult.final_cost = geometricPath.cost(optimizationObjective).value();
                        planningResult.path_length = geometricPath.getStateCount();
                    } else {
                        planningResult.final_cost = planningResult.initial_cost;
                    }
                } else {
                    planningResult.final_cost = planningResult.initial_cost;
                }
            } else {
                planningResult.success = false;
                planningResult.error_message = "No solution found within time limit";
            }
            
        } catch (const std::exception &exception) {
            planningResult.success = false;
            planningResult.error_message = std::string("Planning failed: ") + exception.what();
        }
        
        return planningResult;
    }
    
    /**
     * @brief Get the space information
     * @return Shared pointer to space information
     */
    std::shared_ptr<ob::SpaceInformation> getSpaceInformation() const
    {
        return m_spaceInformation;
    }
    
    /**
     * @brief Get the problem definition
     * @return Shared pointer to problem definition
     */
    std::shared_ptr<ob::ProblemDefinition> getProblemDefinition() const
    {
        return m_problemDefinition;
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
         * The factory now properly follows the Open/Closed Principle:
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
     * @brief Create planner
     * 
     *  Note: This method demonstrates the Facade pattern applied to
     * factory access. It hides the complexity of factory singleton access and
     * provides a clean, simple interface for planner creation.
     */
    std::shared_ptr<ob::Planner> createPlannerByName(const std::string &plannerName) {
        return PlannerFactory::getInstance().createPlanner(plannerName, m_spaceInformation);
    }
    

};

} // namespace vamp_ompl