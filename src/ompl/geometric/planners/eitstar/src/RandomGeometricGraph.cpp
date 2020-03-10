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

#include "ompl/geometric/planners/eitstar/RandomGeometricGraph.h"

#include "ompl/base/OptimizationObjective.h"
#include "ompl/util/GeometricEquations.h"

#include "ompl/geometric/planners/eitstar/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            RandomGeometricGraph::RandomGeometricGraph(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo)

              : samples_(8 /* degree */, 4 /* min degree */, 12 /* max degree */, 50 /* max num points per leaf */,
                         500 /* removed cache size */,
                         false /* rebalancing */)  // These are the defaults. Play with them.
              , spaceInfo_(spaceInfo)
              , dimension_(spaceInfo->getStateDimension())
              , unitNBallMeasure_(unitNBallMeasure(spaceInfo->getStateDimension()))
            {
                samples_.setDistanceFunction(
                    [this](const std::shared_ptr<State> &state1, const std::shared_ptr<State> &state2) {
                        return spaceInfo_->distance(state1->state_, state2->state_);
                    });
            }

            void
            RandomGeometricGraph::setProblemDefinition(const std::shared_ptr<ompl::base::ProblemDefinition> &problem)
            {
                problem_ = problem;
                objective_ = problem->getOptimizationObjective();
                sampler_ = objective_->allocInformedStateSampler(problem, std::numeric_limits<unsigned int>::max());
            }

            void RandomGeometricGraph::setRadiusFactor(double factor)
            {
                radiusFactor_ = factor;
            }

            std::shared_ptr<State> RandomGeometricGraph::getStartState() const
            {
                if (startState_)
                {
                    return startState_;
                }
                else
                {
                    throw std::runtime_error("Start state is not set.");
                }
            }

            std::shared_ptr<State> RandomGeometricGraph::getGoalState() const
            {
                if (goalState_)
                {
                    return goalState_;
                }
                else
                {
                    throw std::runtime_error("Goal state is not set.");
                }
            }

            bool RandomGeometricGraph::hasStartState() const
            {
                return static_cast<bool>(startState_);
            }

            bool RandomGeometricGraph::hasGoalState() const
            {
                return static_cast<bool>(goalState_);
            }

            std::vector<std::shared_ptr<State>> RandomGeometricGraph::getSamples() const
            {
                std::vector<std::shared_ptr<State>> samples;
                samples_.list(samples);
                return samples;
            }

            void RandomGeometricGraph::registerInvalidEdge(const Edge &edge) const
            {
                // Remove the edge from the caches (using the erase-remove idiom).
                auto &sourceNeighbors = edge.source->neighbors_.second;
                auto &targetNeighbors = edge.target->neighbors_.second;
                sourceNeighbors.erase(
                    std::remove_if(sourceNeighbors.begin(), sourceNeighbors.end(),
                                   [&edge](const auto &neighbor) { return neighbor->getId() == edge.target->getId(); }),
                    sourceNeighbors.end());
                targetNeighbors.erase(
                    std::remove_if(targetNeighbors.begin(), targetNeighbors.end(),
                                   [&edge](const auto &neighbor) { return neighbor->getId() == edge.source->getId(); }),
                    targetNeighbors.end());
            }

            std::size_t RandomGeometricGraph::getTag() const
            {
                return tag_;
            }

            std::shared_ptr<State> RandomGeometricGraph::setStartState(const ompl::base::State *startState)
            {
                // Ensure we don't already have a start state.
                if (startState_)
                {
                    throw std::runtime_error("Graph already has a start.");
                }

                // Allocate the start state.
                startState_ = std::make_shared<State>(spaceInfo_, objective_);

                // Set the lower bound for the cost to go.
                startState_->setLowerBoundCostToGo(objective_->costToGo(startState, problem_->getGoal().get()));

                // Set the lower bound for the cost to come.
                startState_->setLowerBoundCostToCome(objective_->identityCost());

                // Copy the given state.
                spaceInfo_->copyState(startState_->raw(), startState);

                // Add the start to the set of samples.
                samples_.add(startState_);

                return startState_;
            }

            std::shared_ptr<State> RandomGeometricGraph::setGoalState(const ompl::base::State *goalState)
            {
                // Ensure we don't already have a start state.
                if (goalState_)
                {
                    throw std::runtime_error("Graph already has a goal.");
                }

                // Allocate the goal state.
                goalState_ = std::make_shared<State>(spaceInfo_, objective_);

                // Copy the given state.
                spaceInfo_->copyState(goalState_->raw(), goalState);

                // Set the lower bound cost to go.
                goalState_->setLowerBoundCostToGo(objective_->identityCost());
                goalState_->setEstimatedCostToGo(objective_->identityCost());
                goalState_->setEstimatedEffortToGo(0u);

                // Sanity check the cost-to-go definition.
                assert(objective_->isCostEquivalentTo(objective_->costToGo(goalState, problem_->getGoal().get()),
                                                      objective_->identityCost()));

                // Add the goal to the set of samples.
                samples_.add(goalState_);

                return goalState_;
            }

            void RandomGeometricGraph::addStates(std::size_t numNewStates)
            {
                // Assert sanity of used variables.
                assert(sampler_);
                assert(objective_);

                // Count the number of informed states before adding new states. This saves some counting, because all
                // new states will be informed, and its known how many of them will be added. If pruning is enabled, we
                // can do this now. After pruning all remaining states are in the informed set.
                auto numInformedSamples{0u};
                if (isPruningEnabled_)
                {
                    prune();
                    numInformedSamples = samples_.size();
                }
                else
                {
                    numInformedSamples = countSamplesInInformedSet();
                }

                // Get the current best cost.
                auto currentCost = goalState_->hasForwardVertex() ? goalState_->asForwardVertex()->getCost() :
                                                                    objective_->infiniteCost();

                // Create the requested number of new states.
                std::vector<std::shared_ptr<State>> newStates;
                newStates.reserve(numNewStates);
                while (newStates.size() < numNewStates)
                {
                    // Allocate a new state.
                    newStates.emplace_back(std::make_shared<State>(spaceInfo_, objective_));
                    auto &newState = newStates.back();

                    do  // Sample randomly until a valid state is found.
                    {
                        sampler_->sampleUniform(newState->raw(), currentCost);
                    } while (!spaceInfo_->isValid(newState->raw()));

                    // Set the lower bound for the cost to go.
                    newState->setLowerBoundCostToGo(objective_->costToGo(newState->raw(), problem_->getGoal().get()));

                    // Set the lower bound for the cost to come.
                    for (std::size_t i = 0u; i < problem_->getStartStateCount(); ++i)
                    {
                        if (objective_->isCostBetterThan(
                                objective_->motionCostHeuristic(newState->raw(), problem_->getStartState(i)),
                                newState->getLowerBoundCostToCome()))
                        {
                            newState->setLowerBoundCostToCome(
                                objective_->motionCostHeuristic(newState->raw(), problem_->getStartState(i)));
                        }
                    }
                }

                // Add the new states to the samples.
                samples_.add(newStates);

                // Update the radius by considering all informed states.
                radius_ = computeRadius(numInformedSamples + numNewStates);

                // Update the tag.
                ++tag_;
            }

            void RandomGeometricGraph::enablePruning(bool prune)
            {
                isPruningEnabled_ = prune;
            }

            const std::vector<std::shared_ptr<State>> &
            RandomGeometricGraph::getNeighbors(const std::shared_ptr<State> &state) const
            {
                assert(state);
                // If the neighbors cache of the vertex isn't up to date, update it.
                if (state->neighbors_.first != tag_)
                {
                    // The cache is invalid, let's clear all vertices.
                    state->neighbors_.second.clear();

                    // Get the neighbors by performing a nearest neighbor search.
                    std::vector<std::shared_ptr<State>> neighbors;
                    samples_.nearestR(state, radius_, neighbors);

                    // We dont want to connect to blacklisted neighbors and the querying state itself.
                    auto connectionPredicate = [&state](const std::shared_ptr<State> &neighbor) {
                        return (state->blacklist_.find(neighbor->id_) == state->blacklist_.end()) &&
                               (state->id_ != neighbor->id_);
                    };

                    // Cache the neighbors that are not blacklisted and not the state itself.
                    std::copy_if(neighbors.begin(), neighbors.end(), std::back_inserter(state->neighbors_.second),
                                 connectionPredicate);

                    // Update the tag of the cache.
                    state->neighbors_.first = tag_;
                }

                // The cache is guaranteed to be up to date now, just return it.
                return state->neighbors_.second;
            }

            void RandomGeometricGraph::prune()
            {
                // Is there really no way of doing this without looping over all samples?
                std::vector<std::shared_ptr<State>> samples;
                samples_.list(samples);

                // Prepare a vector of samples to be pruned.
                std::vector<std::shared_ptr<State>> samplesToBePruned;

                // Check each sample if it can be pruned.
                for (const auto &sample : samples)
                {
                    if (!canPossiblyImproveSolution(sample))
                    {
                        samplesToBePruned.emplace_back(sample);
                    }
                }

                // Remove all samples to be pruned.
                for (const auto &sample : samplesToBePruned)
                {
                    // Remove the sample from the graph.
                    samples_.remove(sample);

                    // Remove it from both search trees.
                    if (sample->hasForwardVertex())
                    {
                        auto forwardVertex = sample->asForwardVertex();
                        if (auto parent = forwardVertex->getParent().lock())
                        {
                            forwardVertex->resetParent();
                            parent->removeChild(forwardVertex);
                        }
                    }
                    if (sample->hasReverseVertex())
                    {
                        auto reverseVertex = sample->asReverseVertex();
                        if (auto parent = reverseVertex->getParent().lock())
                        {
                            reverseVertex->resetParent();
                            parent->removeChild(reverseVertex);
                        }
                    }
                }

                // If any sample was pruned, this has invalidated the nearest neighbor cache.
                if (!samplesToBePruned.empty())
                {
                    ++tag_;
                }
            }

            std::size_t RandomGeometricGraph::countSamplesInInformedSet() const
            {
                // Is there really no way of doing this without looping over all samples?
                std::vector<std::shared_ptr<State>> samples;
                samples_.list(samples);

                // Count the number of samples that can possibly improve the solution and subtract the start and goal
                // states from this number, as they're not technically uniformly distributed.
                return std::count_if(samples.begin(), samples.end(),
                                     [this](const auto &sample) { return canPossiblyImproveSolution(sample); }) -
                       2u;
            }

            bool RandomGeometricGraph::canPossiblyImproveSolution(const std::shared_ptr<State> &state) const
            {
                if (goalState_->hasForwardVertex())
                {
                    return objective_->isCostBetterThan(
                        objective_->combineCosts(objective_->motionCostHeuristic(startState_->raw(), state->raw()),
                                                 objective_->motionCostHeuristic(state->raw(), goalState_->raw())),
                        goalState_->asForwardVertex()->getCost());
                }
                else
                {
                    return true;
                }
            }

            double RandomGeometricGraph::computeRadius(std::size_t numInformedSamples) const
            {
                // Get the solution cost.
                auto solutionCost = goalState_->hasForwardVertex() ? goalState_->asForwardVertex()->getCost() :
                                                                     objective_->infiniteCost();

                // FMT*
                return 2.0 * radiusFactor_ *
                       std::pow((1.0 / dimension_) * (sampler_->getInformedMeasure(solutionCost) / unitNBallMeasure_) *
                                    (std::log(static_cast<double>(numInformedSamples)) / numInformedSamples),
                                1.0 / dimension_);

                // Compute and return the radius. Note to self: double / int -> double. You looked it up. It's fine.
                // RRT*
                // return radiusFactor_ *
                //        std::pow(2.0 * (1.0 + 1.0 / dimension_) *
                //                     (sampler_->getInformedMeasure(solutionCost) / unitNBallMeasure_) *
                //                     (std::log(static_cast<double>(numInformedSamples)) / numInformedSamples),
                //                 1.0 / dimension_);
            }

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl
