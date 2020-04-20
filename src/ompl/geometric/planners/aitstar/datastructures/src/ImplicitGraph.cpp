/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-present University of Oxford
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
 *   * Neither the names of the copyright holders nor the names of its
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
#include "ompl/geometric/planners/aitstar/datastructures/ImplicitGraph.h"

#include <cmath>

#include "ompl/util/GeometricEquations.h"

namespace ompl
{
    namespace geometric
    {
        namespace aitstar
        {
            ImplicitGraph::ImplicitGraph() : batchId_(std::make_shared<std::size_t>(1u))
            {
                // Set the distance function to the space information distance function.
                vertices_.setDistanceFunction(
                    [this](const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) {
                        return spaceInformation_->distance(a->getState(), b->getState());
                    });
            }

            void ImplicitGraph::setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                                      const ompl::base::ProblemDefinitionPtr &problemDefinition,
                                      const std::shared_ptr<ompl::base::Cost> &solutionCost,
                                      const std::shared_ptr<std::size_t> &forwardSearchId,
                                      const std::shared_ptr<std::size_t> &backwardSearchId,
                                      ompl::base::PlannerInputStates* inputStates)
            {
                spaceInformation_ = spaceInformation;
                problemDefinition_ = problemDefinition;
                objective_ = problemDefinition->getOptimizationObjective();
                solutionCost_ = solutionCost;
                forwardSearchId_ = forwardSearchId;
                backwardSearchId_ = backwardSearchId;
                sampler_ =
                    objective_->allocInformedStateSampler(problemDefinition, std::numeric_limits<unsigned int>::max());
                updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition(), inputStates);
            }

            void ImplicitGraph::setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
            }

            void ImplicitGraph::registerStartState(const ompl::base::State *const startState)
            {
                // Create a vertex corresponding to this state.
                auto startVertex = std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_,
                                                            forwardSearchId_, backwardSearchId_);

                // Copy the state into the vertex's state.
                spaceInformation_->copyState(startVertex->getState(), startState);

                // By definition, this has identity cost-to-come.
                startVertex->setCostToComeFromStart(objective_->identityCost());

                // Add the start vertex to the set of vertices.
                vertices_.add(startVertex);

                // Remember it as a start vertex.
                startVertices_.emplace_back(startVertex);
            }

            void ImplicitGraph::registerGoalState(const ompl::base::State *const goalState)
            {
                // Create a vertex corresponding to this state.
                auto goalVertex = std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_,
                                                           forwardSearchId_, backwardSearchId_);

                // Copy the state into the vertex's state.
                spaceInformation_->copyState(goalVertex->getState(), goalState);

                // Add the goal vertex to the set of vertices.
                vertices_.add(goalVertex);

                // Remember it as a goal vertex.
                goalVertices_.emplace_back(goalVertex);
            }

            bool ImplicitGraph::hasAStartState() const
            {
                return !startVertices_.empty();
            }

            bool ImplicitGraph::hasAGoalState() const
            {
                return !goalVertices_.empty();
            }

            void
            ImplicitGraph::updateStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition,
                                                    ompl::base::PlannerInputStates *inputStates)
            {
                // We need to keep track whether a new goal and/or a new start has been added.
                bool addedNewGoalState = false;
                bool addedNewStartState = false;

                // First update the goals. We have to call inputStates->nextGoal(terminationCondition) at least once
                // (regardless of the return value of inputStates->moreGoalStates()) in case the termination condition
                // wants us to wait for a goal.
                do
                {
                    // Get a new goal. If there are none, or the underlying state is invalid this will be a nullptr.
                    auto newGoalState = inputStates->nextGoal(terminationCondition);

                    // If there was a new valid goal, register it as such and remember that a goal has been added.
                    if (static_cast<bool>(newGoalState))
                    {
                        registerGoalState(newGoalState);
                        addedNewGoalState = true;
                    }

                } while (inputStates->haveMoreGoalStates());

                // Having updated the goals, we now update the starts.
                while (inputStates->haveMoreStartStates())
                {
                    // Get the next start. The returned pointer can be a nullptr (if the state is invalid).
                    auto newStartState = inputStates->nextStart();

                    // If there is a new valid start, register it as such and remember that a start has been added.
                    if (static_cast<bool>(newStartState))
                    {
                        registerStartState(newStartState);
                        addedNewStartState = true;
                    }
                }

                // If we added a new start and have previously pruned goals, we might want to add the goals back.
                if (addedNewStartState && !prunedGoalVertices_.empty())
                {
                    // Keep track of the pruned goal vertices that have been revived.
                    std::vector<std::vector<std::shared_ptr<Vertex>>::iterator> revivedGoals;

                    // Let's see if the pruned goal is close enough to any new start to revive it..
                    for (auto it = prunedGoalVertices_.begin(); it != prunedGoalVertices_.end(); ++it)
                    {
                        // Loop over all start states to get the best cost.
                        auto heuristicCost = objective_->infiniteCost();
                        for (const auto &start : startVertices_)
                        {
                            heuristicCost = objective_->betterCost(
                                heuristicCost, objective_->motionCostHeuristic(start->getState(), (*it)->getState()));
                        }

                        // If this goal can possibly improve the current solution, add it back to the graph.
                        if (objective_->isCostBetterThan(heuristicCost, *solutionCost_.lock()))
                        {
                            registerGoalState((*it)->getState());
                            addedNewGoalState = true;
                            revivedGoals.emplace_back(it);
                        }
                    }

                    // Remove all revived goals from the pruned goals.
                    for (const auto &revivedGoal : revivedGoals)
                    {
                        std::iter_swap(revivedGoal, prunedGoalVertices_.rbegin());
                        prunedGoalVertices_.pop_back();
                    }
                }

                // If we added a new goal and have previously pruned starts, we might want to add the starts back.
                if (addedNewGoalState && !prunedStartVertices_.empty())
                {
                    // Keep track of the pruned goal vertices that have been revived.
                    std::vector<std::vector<std::shared_ptr<Vertex>>::iterator> revivedStarts;

                    // Let's see if the pruned start is close enough to any new goal to revive it..
                    for (auto it = prunedStartVertices_.begin(); it != prunedStartVertices_.end(); ++it)
                    {
                        // Loop over all start states to get the best cost.
                        auto heuristicCost = objective_->infiniteCost();
                        for (const auto &goal : goalVertices_)
                        {
                            heuristicCost = objective_->betterCost(
                                heuristicCost, objective_->motionCostHeuristic(goal->getState(), (*it)->getState()));
                        }

                        // If this goal can possibly improve the current solution, add it back to the graph.
                        if (objective_->isCostBetterThan(heuristicCost, *solutionCost_.lock()))
                        {
                            registerStartState((*it)->getState());
                            addedNewStartState = true;
                            revivedStarts.emplace_back(it);
                        }
                    }

                    // Remove all revived goals from the pruned goals.
                    for (const auto &revivedStart : revivedStarts)
                    {
                        std::iter_swap(revivedStart, prunedStartVertices_.rbegin());
                        prunedGoalVertices_.pop_back();
                    }
                }

                if (addedNewGoalState || addedNewStartState)
                {
                    // Allocate a state sampler if we have at least one start and one goal.
                    if (!startVertices_.empty() && !goalVertices_.empty())
                    {
                        sampler_ = objective_->allocInformedStateSampler(problemDefinition_,
                                                                         std::numeric_limits<unsigned int>::max());
                    }
                }

                if (!goalVertices_.empty() && startVertices_.empty())
                {
                    OMPL_WARN("AIT* (ImplicitGraph): The problem has a goal but not a start. AIT* can not find a "
                              "solution since PlannerInputStates provides no method to wait for a valid start state to "
                              "appear.");
                }
            }

            std::size_t ImplicitGraph::computeNumberOfSamplesInInformedSet() const
            {
                std::size_t numberOfSamplesInInformedSet{0u};
                std::vector<std::shared_ptr<Vertex>> vertices;
                vertices_.list(vertices);

                // Loop over all vertices.
                for (const auto &vertex : vertices)
                {
                    // Get the best cost to come from any start.
                    ompl::base::Cost bestCostToComeHeuristic = objective_->infiniteCost();
                    for (const auto &start : startVertices_)
                    {
                        auto costToComeHeuristic =
                            objective_->motionCostHeuristic(start->getState(), vertex->getState());
                        if (objective_->isCostBetterThan(costToComeHeuristic, bestCostToComeHeuristic))
                        {
                            bestCostToComeHeuristic = costToComeHeuristic;
                        }
                    }

                    // Get the best cost to go to any goal.
                    ompl::base::Cost bestCostToGoHeuristic = objective_->infiniteCost();
                    for (const auto &goal : goalVertices_)
                    {
                        auto costToComeHeuristic =
                            objective_->motionCostHeuristic(vertex->getState(), goal->getState());
                        if (objective_->isCostBetterThan(costToComeHeuristic, bestCostToGoHeuristic))
                        {
                            bestCostToGoHeuristic = costToComeHeuristic;
                        }
                    }

                    // If this can possibly improve the current solution, it is in the informed set.
                    if (objective_->isCostBetterThan(
                            objective_->combineCosts(bestCostToComeHeuristic, bestCostToGoHeuristic),
                            *solutionCost_.lock()))
                    {
                        ++numberOfSamplesInInformedSet;
                    }
                }

                return numberOfSamplesInInformedSet;
            }

            std::vector<std::shared_ptr<Vertex>> ImplicitGraph::addSamples(std::size_t numNewSamples)
            {
                // First get the number of samples inside the informed set.
                auto numSamplesInInformedSet = computeNumberOfSamplesInInformedSet();

                // Create new vertices.
                std::vector<std::shared_ptr<Vertex>> newVertices;
                newVertices.reserve(numNewSamples);
                while (newVertices.size() < numNewSamples)
                {
                    // Create a new vertex.
                    newVertices.emplace_back(std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_,
                                                                      forwardSearchId_, backwardSearchId_));

                    do
                    {
                        // Sample the associated state uniformly within the informed set.
                        sampler_->sampleUniform(newVertices.back()->getState(), *solutionCost_.lock());
                    } while (!spaceInformation_->getStateValidityChecker()->isValid(newVertices.back()->getState()));
                }

                // Add all new vertices to the nearest neighbor structure.
                vertices_.add(newVertices);

                // We need to do some internal housekeeping.
                ++(*batchId_);
                radius_ = computeConnectionRadius(numSamplesInInformedSet + numNewSamples - startVertices_.size() -
                                                  goalVertices_.size());

                return newVertices;
            }

            std::size_t ImplicitGraph::getNumVertices() const
            {
                return vertices_.size();
            }

            double ImplicitGraph::getConnectionRadius() const
            {
                return radius_;
            }

            std::vector<std::shared_ptr<Vertex>>
            ImplicitGraph::getNeighbors(const std::shared_ptr<Vertex> &vertex) const
            {
                // Return cached neighbors if available.
                if (vertex->hasCachedNeighbors())
                {
                    return vertex->getNeighbors();
                }
                else
                {
                    std::vector<std::shared_ptr<Vertex>> neighbors{};
                    vertices_.nearestR(vertex, radius_, neighbors);
                    vertex->cacheNeighbors(neighbors);
                    return neighbors;
                }
            }

            bool ImplicitGraph::isStart(const std::shared_ptr<Vertex> &vertex) const
            {
                for (const auto &start : startVertices_)
                {
                    if (vertex->getId() == start->getId())
                    {
                        return true;
                    }
                }
                return false;
            }

            bool ImplicitGraph::isGoal(const std::shared_ptr<Vertex> &vertex) const
            {
                for (const auto &goal : goalVertices_)
                {
                    if (vertex->getId() == goal->getId())
                    {
                        return true;
                    }
                }
                return false;
            }

            const std::vector<std::shared_ptr<Vertex>> &ImplicitGraph::getStartVertices() const
            {
                return startVertices_;
            }

            const std::vector<std::shared_ptr<Vertex>> &ImplicitGraph::getGoalVertices() const
            {
                return goalVertices_;
            }

            std::vector<std::shared_ptr<Vertex>> ImplicitGraph::getVertices() const
            {
                std::vector<std::shared_ptr<Vertex>> vertices;
                vertices_.list(vertices);
                return vertices;
            }

            void ImplicitGraph::prune()
            {
                if (!objective_->isFinite(*(solutionCost_.lock())))
                {
                    return;
                }

                std::vector<std::shared_ptr<Vertex>> vertices;
                vertices_.list(vertices);

                // Prepare the vector of vertices to be pruned.
                std::vector<std::shared_ptr<Vertex>> verticesToBePruned;

                // Check each vertex whether it can be pruned.
                for (const auto &vertex : vertices)
                {
                    // Check if the combination of the admissible costToCome and costToGo estimates results in a path
                    // that is more expensive than the current solution.
                    if (!canPossiblyImproveSolution(vertex))
                    {
                        // We keep track of pruned start and goal vertices. This is because if the user adds start or
                        // goal states after we have pruned start or goal states, we might want to reconsider pruned
                        // start or goal states.
                        if (isGoal(vertex))
                        {
                            prunedGoalVertices_.emplace_back(vertex);
                        }
                        else if (isStart(vertex))
                        {
                            prunedStartVertices_.emplace_back(vertex);
                        }
                        verticesToBePruned.emplace_back(vertex);
                    }
                }

                // Remove all vertices to be pruned.
                for (const auto &vertex : verticesToBePruned)
                {
                    // Remove it from both search trees.
                    if (vertex->hasBackwardParent())
                    {
                        vertex->getBackwardParent()->removeFromBackwardChildren(vertex->getId());
                        vertex->resetBackwardParent();
                    }
                    vertex->invalidateBackwardBranch();
                    if (vertex->hasForwardParent())
                    {
                        vertex->getForwardParent()->removeFromForwardChildren(vertex->getId());
                        vertex->resetForwardParent();
                    }
                    vertex->invalidateForwardBranch();

                    // Remove it from the nearest neighbor struct.
                    vertices_.remove(vertex);
                }

                // Assert that the forward and reverse queue are empty?
            }

            double ImplicitGraph::computeConnectionRadius(std::size_t numSamples) const
            {
                // Define the dimension as a helper variable.
                auto dimension = static_cast<double>(spaceInformation_->getStateDimension());

                // Compute the RRT* factor.
                // return rewireFactor_ *
                //        std::pow(2.0 * (1.0 + 1.0 / dimension) *
                //                     (sampler_->getInformedMeasure(*solutionCost_.lock()) /
                //                      unitNBallMeasure(spaceInformation_->getStateDimension())) *
                //                     (std::log(static_cast<double>(numSamples)) / static_cast<double>(numSamples)),
                //                 1.0 / dimension);

                // Compute the FMT* factor.
                return 2.0 * rewireFactor_ *
                       std::pow((1.0 / dimension) *
                                    (sampler_->getInformedMeasure(*solutionCost_.lock()) /
                                     unitNBallMeasure(spaceInformation_->getStateDimension())) *
                                    (std::log(static_cast<double>(numSamples)) / numSamples),
                                1.0 / dimension);
            }

            bool ImplicitGraph::canPossiblyImproveSolution(const std::shared_ptr<Vertex> &vertex) const
            {
                // Get the preferred start for this vertex.
                auto bestCostToCome = objective_->infiniteCost();
                for (const auto &start : startVertices_)
                {
                    auto costToCome = objective_->motionCostHeuristic(start->getState(), vertex->getState());
                    if (objective_->isCostBetterThan(costToCome, bestCostToCome))
                    {
                        bestCostToCome = costToCome;
                    }
                }

                // Check if the combination of the admissible costToCome and costToGo estimates results in a path
                // that is more expensive than the current solution.
                return objective_->isCostBetterThan(
                    objective_->combineCosts(
                        bestCostToCome, objective_->costToGo(vertex->getState(), problemDefinition_->getGoal().get())),
                    *(solutionCost_.lock()));
            }

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl
