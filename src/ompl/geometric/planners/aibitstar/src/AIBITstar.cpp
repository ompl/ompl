/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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
 *   * Neither the name of the University of Toronto nor the names of its
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

// Authors: Marlin Strub

#include "ompl/geometric/planners/aibitstar/AIBITstar.h"

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

namespace ompl
{
    namespace geometric
    {
        using namespace aibitstar;

        AIBITstar::AIBITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo)
          : ompl::base::Planner(spaceInfo, "AI-BIT*")
          , graph_(spaceInfo)
          , motionValidator_(spaceInfo->getMotionValidator())
        {
        }

        void AIBITstar::setup()
        {
            // Check that the problem definition is set.
            if (!problem_)
            {
                OMPL_ERROR("AI-BIT* can not be setup without first setting the probelm definition.");
                return;
            }

            // Check the goal is of appropriate type.
            if (!problem_->getGoal()->hasType(ompl::base::GOAL_STATE))
            {
                OMPL_ERROR("AI-BIT* currently only works for single goal states.");
                return;
            }

            // Call the base class setup.
            Planner::setup();

            // Default to path length optimization if no objective has been specified.
            if (!problem_->hasOptimizationObjective())
            {
                OMPL_WARN("%s: No optimization has been specified. Defaulting to path length.", name_.c_str());
                problem_->setOptimizationObjective(
                    std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_));
            }

            // Pull through the optimization objective for direct access.
            objective_ = problem_->getOptimizationObjective();

            // Let the graph know about the objective such that it can focus its approximation.
            graph_.setOptimizationObjective(objective_);

            // Add the start state.
            while (Planner::pis_.haveMoreStartStates())
            {
                if (const auto start = Planner::pis_.nextStart())
                {
                    auto startState = graph_.setStartState(start);
                    assert(graph_.hasStartState());
                    forwardRoot_ = std::make_shared<Vertex>(startState);
                    startState->setForwardVertex(forwardRoot_);
                }
            }

            // Add the goal state.
            while (Planner::pis_.haveMoreGoalStates())
            {
                if (const auto goal = Planner::pis_.nextGoal())
                {
                    auto goalState = graph_.setGoalState(goal);
                    assert(graph_.hasGoalState());
                    reverseRoot_ = std::make_shared<Vertex>(goalState);
                    goalState->setReverseVertex(reverseRoot_);
                }
            }
        }

        ompl::base::PlannerStatus AIBITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Make sure everything is setup.
            if (!setup_)
            {
                throw std::runtime_error("Called solve on AIBIT* without setting up the planner first.");
            }
            if (!spaceInfo_->isSetup())
            {
                throw std::runtime_error("Called solve on AIBIT* without setting up the state space first.");
            }

            // If this is the first time solve is being called, populate the backward queue.
            if (iteration_ == 0u)
            {
                reverseExpand(graph_.getGoalState());
            }
        }

        std::vector<Edge> AIBITstar::reverseExpand(const std::shared_ptr<State> &state) const
        {
            // Expanding a state into the reverse queue must mean the state has a parent vertex in the reverse tree.
            assert(state->getReverseVertex().lock());

            // Prepare the return values.
            std::vector<Edge> outgoingEdges;

            // Get the reverse vertex.
            auto vertex = state->getReverseVertex().lock();

            // Get the neighbors in the current graph.
            for (const auto &neighborState : graph_.getNeighbors(state))
            {
                // Compute the heuristic motion cost of this edge.
                auto heuristicCost = objective_->motionCostHeuristic(state->getState(), neighborState->getState());

                // If this state is already associated with a vertex in the reverse tree, use this vertex, otherwise
                // create a new one.
                if (auto neighbor = neighborState->getReverseVertex().lock())
                {
                    outgoingEdges.emplace_back(vertex, neighbor, heuristicCost,
                                               computeReverseKey(vertex, neighbor, heuristicCost));
                }
                else
                {
                    neighbor = std::make_shared<Vertex>(neighborState);
                    neighborState->setReverseVertex(neighbor);
                    outgoingEdges.emplace_back(vertex, neighbor, heuristicCost,
                                               computeReverseKey(vertex, neighbor, heuristicCost));
                }
            }

            // Get the forward parent and children.
            std::shared_ptr<Vertex> forwardParent;
            std::vector<std::shared_ptr<Vertex>> forwardChildren;
            if (auto forwardVertex = state->getForwardVertex().lock())
            {
                assert(forwardVertex->getParent().lock());
                forwardChildren = forwardVertex->getChildren();
            }

            return outgoingEdges;
        }

        std::array<double, 3u> AIBITstar::computeReverseKey(const std::shared_ptr<Vertex> &parent,
                                                            const std::shared_ptr<Vertex> &child,
                                                            const ompl::base::Cost &edgeCost) const
        {
            return {objective_
                        ->combineCosts(parent->getCost(),
                                       objective_->combineCosts(edgeCost, objective_->motionCostHeuristic(
                                                                              child->getState()->getState(),
                                                                              forwardRoot_->getState()->getState())))
                        .value(),
                    objective_->combineCosts(parent->getCost(), edgeCost).value(), parent->getCost().value()};
        }

    }  // namespace geometric
}  // namespace ompl
