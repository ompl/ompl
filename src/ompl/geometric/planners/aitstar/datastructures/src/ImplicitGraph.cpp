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
                                      const std::shared_ptr<std::size_t> &backwardSearchId)
            {
                spaceInformation_ = spaceInformation;
                problemDefinition_ = problemDefinition;
                optimizationObjective_ = problemDefinition->getOptimizationObjective();
                solutionCost_ = solutionCost;
                forwardSearchId_ = forwardSearchId;
                backwardSearchId_ = backwardSearchId;
                sampler_ = optimizationObjective_->allocInformedStateSampler(problemDefinition,
                                                                             std::numeric_limits<unsigned int>::max());
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
                startVertex->setCostToComeFromStart(optimizationObjective_->identityCost());

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

            std::size_t ImplicitGraph::computeNumberOfSamplesInInformedSet() const
            {
                std::size_t numberOfSamplesInInformedSet{0u};
                std::vector<std::shared_ptr<Vertex>> vertices;
                vertices_.list(vertices);

                // Loop over all vertices.
                for (const auto &vertex : vertices)
                {
                    // Get the best cost to come from any start.
                    ompl::base::Cost bestCostToComeHeuristic = optimizationObjective_->infiniteCost();
                    for (const auto &start : startVertices_)
                    {
                        auto costToComeHeuristic =
                            optimizationObjective_->motionCostHeuristic(start->getState(), vertex->getState());
                        if (optimizationObjective_->isCostBetterThan(costToComeHeuristic, bestCostToComeHeuristic))
                        {
                            bestCostToComeHeuristic = costToComeHeuristic;
                        }
                    }

                    // Get the best cost to go to any goal.
                    ompl::base::Cost bestCostToGoHeuristic = optimizationObjective_->infiniteCost();
                    for (const auto &goal : goalVertices_)
                    {
                        auto costToComeHeuristic =
                            optimizationObjective_->motionCostHeuristic(vertex->getState(), goal->getState());
                        if (optimizationObjective_->isCostBetterThan(costToComeHeuristic, bestCostToGoHeuristic))
                        {
                            bestCostToGoHeuristic = costToComeHeuristic;
                        }
                    }

                    // If this can possibly improve the current solution, it is in the informed set.
                    if (optimizationObjective_->isCostBetterThan(
                            optimizationObjective_->combineCosts(bestCostToComeHeuristic, bestCostToGoHeuristic),
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
                if (!optimizationObjective_->isFinite(*(solutionCost_.lock())))
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
                    // Get the preferred start for this vertex.
                    auto bestCostToCome = optimizationObjective_->infiniteCost();
                    for (const auto &start : startVertices_)
                    {
                        auto costToCome =
                            optimizationObjective_->motionCostHeuristic(start->getState(), vertex->getState());
                        if (optimizationObjective_->isCostBetterThan(costToCome, bestCostToCome))
                        {
                            bestCostToCome = costToCome;
                        }
                    }

                    // Check if the combination of the admissible costToCome and costToGo estimates results in a path
                    // that is more expensive than the current solution.
                    if (!optimizationObjective_->isCostBetterThan(
                            optimizationObjective_->combineCosts(
                                bestCostToCome, optimizationObjective_->costToGo(vertex->getState(),
                                                                                 problemDefinition_->getGoal().get())),
                            *(solutionCost_.lock())))
                    {
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

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl
