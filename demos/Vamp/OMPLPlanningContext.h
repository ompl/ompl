#pragma once

#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <chrono>
#include <iostream>

namespace vamp_ompl {

namespace og = ompl::geometric;

/**
 * @brief OMPL planning context that manages OMPL-specific setup and planning
 * 
 * This class is responsible for:
 * - Setting up OMPL state spaces and bounds
 * - Creating and configuring planners
 * - Managing problem definitions
 * - Running planning and simplification
 */
template<typename Robot>
class OMPLPlanningContext {
public:
    using Configuration = typename Robot::Configuration;
    static constexpr std::size_t dimension = Robot::dimension;
    static constexpr std::size_t rake = vamp::FloatVectorWidth;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

private:
    std::shared_ptr<ob::SpaceInformation> si_;
    std::shared_ptr<ob::ProblemDefinition> pdef_;
    
public:
    /**
     * @brief Setup the OMPL state space using robot configuration
     * @param robot_config Robot configuration providing joint limits
     * @param env_v VAMP environment for validators
     */
    void setupStateSpace(const RobotConfig<Robot> &robot_config, const EnvironmentVector &env_v)
    {
        // Create state space
        auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
        
        // Set joint limits
        ob::RealVectorBounds bounds(dimension);
        auto joint_limits = robot_config.getJointLimits();
        
        if (joint_limits.size() != dimension) {
            throw std::runtime_error("Joint limits size (" + std::to_string(joint_limits.size()) + 
                                   ") does not match robot dimension (" + std::to_string(dimension) + ")");
        }
        
        for (size_t i = 0; i < dimension; ++i) {
            bounds.setLow(i, joint_limits[i].first);
            bounds.setHigh(i, joint_limits[i].second);
        }
        
        space->setBounds(bounds);
        
        // Create space information
        si_ = std::make_shared<ob::SpaceInformation>(space);
        
        // Set VAMP validators
        si_->setStateValidityChecker(std::make_shared<VampStateValidator<Robot>>(si_, env_v));
        si_->setMotionValidator(std::make_shared<VampMotionValidator<Robot>>(si_, env_v));
        
        si_->setup();
    }
    
    /**
     * @brief Set up the planning problem with start and goal configurations
     * @param start_config Start configuration as array
     * @param goal_config Goal configuration as array
     */
    void setProblem(const std::array<float, dimension> &start_config, 
                   const std::array<float, dimension> &goal_config)
    {
        if (!si_) {
            throw std::runtime_error("State space not set up. Call setupStateSpace first.");
        }
        
        auto space = si_->getStateSpace();
        ob::ScopedState<> start_ompl(space), goal_ompl(space);
        
        // Convert arrays to OMPL states directly
        for (size_t i = 0; i < dimension; ++i) {
            start_ompl[i] = static_cast<double>(start_config[i]);
            goal_ompl[i] = static_cast<double>(goal_config[i]);
        }
        
        // Create problem definition
        pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
        pdef_->setStartAndGoalStates(start_ompl, goal_ompl);
        
        // Set optimization objective
        auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
        pdef_->setOptimizationObjective(obj);
    }
    
    /**
     * @brief Plan using the specified configuration
     * @param config Planning configuration
     * @return Planning results with timing and path information
     */
    PlanningResult plan(const PlanningConfig &config)
    {
        if (!pdef_) {
            throw std::runtime_error("Problem not defined. Call setProblem first.");
        }
        
        PlanningResult result;
        
        try {
            // Create planner
            auto planner = createPlanner(config.planner_name);
            planner->setProblemDefinition(pdef_);
            planner->setup();
            
            // Set optimization threshold if not optimizing
            if (!config.optimize_path) {
                auto obj = pdef_->getOptimizationObjective();
                obj->setCostThreshold(obj->infiniteCost());
            }
            
            // Plan
            auto start_time = std::chrono::steady_clock::now();
            ob::PlannerStatus solved = planner->solve(config.planning_time);
            auto end_time = std::chrono::steady_clock::now();
            
            result.planning_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time).count();
            
            // Check if solution was found
            if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
                result.success = true;
                result.solution_path = pdef_->getSolutionPath();
                
                // Get initial cost
                og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*result.solution_path);
                auto obj = pdef_->getOptimizationObjective();
                result.initial_cost = path_geometric.cost(obj).value();
                result.path_length = path_geometric.getStateCount();
                
                // Simplify path if requested
                if (config.simplification_time > 0.0) {
                    auto simplify_start = std::chrono::steady_clock::now();
                    
                    og::PathSimplifier simplifier(si_, pdef_->getGoal(), obj);
                    bool simplified = simplifier.simplify(path_geometric, config.simplification_time);
                    
                    auto simplify_end = std::chrono::steady_clock::now();
                    result.simplification_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        simplify_end - simplify_start).count();
                    
                    if (simplified) {
                        result.final_cost = path_geometric.cost(obj).value();
                        result.path_length = path_geometric.getStateCount();
                    } else {
                        result.final_cost = result.initial_cost;
                    }
                } else {
                    result.final_cost = result.initial_cost;
                }
            } else {
                result.success = false;
                result.error_message = "No solution found within time limit";
            }
            
        } catch (const std::exception &e) {
            result.success = false;
            result.error_message = std::string("Planning failed: ") + e.what();
        }
        
        return result;
    }
    
    /**
     * @brief Get the space information
     * @return Shared pointer to space information
     */
    std::shared_ptr<ob::SpaceInformation> getSpaceInformation() const
    {
        return si_;
    }
    
    /**
     * @brief Get the problem definition
     * @return Shared pointer to problem definition
     */
    std::shared_ptr<ob::ProblemDefinition> getProblemDefinition() const
    {
        return pdef_;
    }

private:
    /**
     * @brief Create a planner by name
     * @param planner_name Name of the planner ("BIT*", "RRT-Connect", "PRM")
     * @return Shared pointer to created planner
     */
    std::shared_ptr<ob::Planner> createPlanner(const std::string &planner_name)
    {
        if (planner_name == "BIT*") {
            return std::make_shared<og::BITstar>(si_);
        } else if (planner_name == "RRT-Connect") {
            return std::make_shared<og::RRTConnect>(si_);
        } else if (planner_name == "PRM") {
            return std::make_shared<og::PRM>(si_);
        } else {
            std::cerr << "Unknown planner: " << planner_name << ". Using BIT* as default." << std::endl;
            return std::make_shared<og::BITstar>(si_);
        }
    }
    

};

} // namespace vamp_ompl