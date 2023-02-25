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
#include "ompl/geometric/planners/informedtrees/aitstar/ImplicitGraph.h"

#include <cmath>

#include <boost/math/constants/constants.hpp>

#include "ompl/util/GeometricEquations.h"

namespace ompl
{
    namespace geometric
    {
        namespace aitstar
        {
            ImplicitGraph::ImplicitGraph(const ompl::base::Cost &solutionCost)
              : batchId_(1u), solutionCost_(solutionCost)
            {
            }

            void ImplicitGraph::setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                                      const ompl::base::ProblemDefinitionPtr &problemDefinition,
                                      ompl::base::PlannerInputStates *inputStates)
            {
                vertices_.setDistanceFunction(
                    [this](const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) {
                        return spaceInformation_->distance(a->getState(), b->getState());
                    });
                spaceInformation_ = spaceInformation;
                problemDefinition_ = problemDefinition;
                objective_ = problemDefinition->getOptimizationObjective();
                k_rgg_ = boost::math::constants::e<double>() +
                         (boost::math::constants::e<double>() / spaceInformation->getStateDimension());
                updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition(), inputStates);
            }

            void ImplicitGraph::clear()
            {
                batchId_ = 1u;
                radius_ = std::numeric_limits<double>::infinity();
                numNeighbors_ = std::numeric_limits<std::size_t>::max();
                vertices_.clear();
                startVertices_.clear();
                goalVertices_.clear();
                prunedStartVertices_.clear();
                prunedGoalVertices_.clear();
                numSampledStates_ = 0u;
                numValidSamples_ = 0u;
            }

            void ImplicitGraph::setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
            }

            double ImplicitGraph::getRewireFactor() const
            {
                return rewireFactor_;
            }

            void ImplicitGraph::setMaxNumberOfGoals(unsigned int maxNumberOfGoals)
            {
                maxNumGoals_ = maxNumberOfGoals;
            }

            unsigned int ImplicitGraph::getMaxNumberOfGoals() const
            {
                return maxNumGoals_;
            }

            void ImplicitGraph::setUseKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            bool ImplicitGraph::getUseKNearest() const
            {
                return useKNearest_;
            }

            void ImplicitGraph::registerStartState(const ompl::base::State *const startState)
            {
                // Create a vertex corresponding to this state.
                auto startVertex = std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_);

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
                auto goalVertex = std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_);

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

                } while (inputStates->haveMoreGoalStates() && goalVertices_.size() <= maxNumGoals_);

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
                        if (objective_->isCostBetterThan(heuristicCost, solutionCost_))
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
                        if (objective_->isCostBetterThan(heuristicCost, solutionCost_))
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
                        prunedStartVertices_.pop_back();
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
                // Loop over all vertices and count the ones in the informed set.
                std::size_t numberOfSamplesInInformedSet{0u};
                for (const auto &vertex : getVertices())
                {
                    // Get the best cost to come from any start.
                    auto costToCome = objective_->infiniteCost();
                    for (const auto &start : startVertices_)
                    {
                        costToCome = objective_->betterCost(
                            costToCome, objective_->motionCostHeuristic(start->getState(), vertex->getState()));
                    }

                    // Get the best cost to go to any goal.
                    auto costToGo = objective_->infiniteCost();
                    for (const auto &goal : goalVertices_)
                    {
                        costToGo = objective_->betterCost(
                            costToCome, objective_->motionCostHeuristic(vertex->getState(), goal->getState()));
                    }

                    // If this can possibly improve the current solution, it is in the informed set.
                    if (objective_->isCostBetterThan(objective_->combineCosts(costToCome, costToGo), solutionCost_))
                    {
                        ++numberOfSamplesInInformedSet;
                    }
                }

                return numberOfSamplesInInformedSet;
            }

            bool ImplicitGraph::addSamples(std::size_t numNewSamples,
                                           const ompl::base::PlannerTerminationCondition &terminationCondition)
            {
                // If there are no states to be added, then there's nothing to do.
                if (numNewSamples == 0u)
                {
                    return true;
                }

                // Ensure there's enough space for the new samples.
                newSamples_.reserve(numNewSamples);

                do
                {
                    // Create a new vertex.
                    newSamples_.emplace_back(std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_));

                    bool foundValidSample = false;
                    do
                    {
                        // Sample the associated state uniformly within the informed set.
                        sampler_->sampleUniform(newSamples_.back()->getState(), solutionCost_);

                        // Count how many states we've checked.
                        ++numSampledStates_;

                        // Check if the sample is valid.
                        foundValidSample = spaceInformation_->getStateValidityChecker()->isValid(newSamples_.back()->getState());
                    } while (!foundValidSample && !terminationCondition);

                    // The sample can be invalid if the termination condition is met.
                    if (foundValidSample)
                    {
                        // If this state happens to satisfy the goal condition, add it as such.
                        if (problemDefinition_->getGoal()->isSatisfied(newSamples_.back()->getState()))
                        {
                            goalVertices_.emplace_back(newSamples_.back());
                            newSamples_.back()->setCostToComeFromGoal(objective_->identityCost());
                        }

                        ++numValidSamples_;
                    }
                    else
                    {
                        // Remove the invalid sample.
                        newSamples_.pop_back();
                    }
                } while (newSamples_.size() < numNewSamples && !terminationCondition);

                if (newSamples_.size() == numNewSamples)
                {
                    // First get the number of samples inside the informed set.
                    auto numSamplesInInformedSet = computeNumberOfSamplesInInformedSet();

                    if (useKNearest_)
                    {
                        numNeighbors_ = computeNumberOfNeighbors(numSamplesInInformedSet + numNewSamples -
                                                                 startVertices_.size() - goalVertices_.size());
                    }
                    else
                    {
                        radius_ = computeConnectionRadius(numSamplesInInformedSet + numNewSamples -
                                                          startVertices_.size() - goalVertices_.size());
                    }

                    // Add all new vertices to the nearest neighbor structure.
                    vertices_.add(newSamples_);
                    newSamples_.clear();

                    // Update the batch id.
                    ++batchId_;

                    return true;
                }

                return false;
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
                    ++numNearestNeighborsCalls_;
                    std::vector<std::shared_ptr<Vertex>> neighbors{};
                    if (useKNearest_)
                    {
                        vertices_.nearestK(vertex, numNeighbors_, neighbors);
                    }
                    else
                    {
                        vertices_.nearestR(vertex, radius_, neighbors);
                    }
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
                if (!objective_->isFinite(solutionCost_))
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
                    if (vertex->hasReverseParent())
                    {
                        vertex->getReverseParent()->removeFromReverseChildren(vertex->getId());
                        vertex->resetReverseParent();
                    }
                    vertex->invalidateReverseBranch();
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

            std::size_t ImplicitGraph::getNumberOfSampledStates() const
            {
                return numSampledStates_;
            }

            std::size_t ImplicitGraph::getNumberOfValidSamples() const
            {
                return numValidSamples_;
            }

            std::size_t ImplicitGraph::getNumberOfStateCollisionChecks() const
            {
                // Each sampled state is checked for collision. Only sampled states are checked for collision (number of
                // collision checked edges don't count here.)
                return numSampledStates_;
            }

            std::size_t ImplicitGraph::getNumberOfNearestNeighborCalls() const
            {
                return numNearestNeighborsCalls_;
            }

            double ImplicitGraph::computeConnectionRadius(std::size_t numSamples) const
            {
                // Define the dimension as a helper variable.
                auto dimension = static_cast<double>(spaceInformation_->getStateDimension());

                // Compute the RRT* factor.
                return rewireFactor_ *
                       std::pow(2.0 * (1.0 + 1.0 / dimension) *
                                    (sampler_->getInformedMeasure(solutionCost_) /
                                     unitNBallMeasure(spaceInformation_->getStateDimension())) *
                                    (std::log(static_cast<double>(numSamples)) / static_cast<double>(numSamples)),
                                1.0 / dimension);

                // // Compute the FMT* factor.
                // return 2.0 * rewireFactor_ *
                //        std::pow((1.0 / dimension) *
                //                     (sampler_->getInformedMeasure(*solutionCost_.lock()) /
                //                      unitNBallMeasure(spaceInformation_->getStateDimension())) *
                //                     (std::log(static_cast<double>(numSamples)) / numSamples),
                //                 1.0 / dimension);
            }

            std::size_t ImplicitGraph::computeNumberOfNeighbors(std::size_t numSamples) const
            {
                return std::ceil(rewireFactor_ * k_rgg_ * std::log(static_cast<double>(numSamples)));
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
                    solutionCost_);
            }

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl
